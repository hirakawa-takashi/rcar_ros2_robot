"""AI-CAR 前方障害物判定ノード。

LiDAR (`/scan`) を主情報として前方セクターの障害物距離と危険度を判定し、
カメラ (`/camera/image_raw/compressed`) の画像を AI HAT+ (Hailo-8) の
YOLO 推論にかけて障害物の種別を補足する。
CPU / AI HAT+ の温度に応じて推論レートを自動的に落とす（サーマルガバナ）。

判定結果は `/obstacle_status`（std_msgs/String, JSON）へ publish する。
"""

import json
import math
import threading
import time
from collections import deque
from contextlib import ExitStack

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

# COCO 80 クラス（YOLOv8 の class_id 順）。表示用に日本語名を持つ。
COCO_CLASSES = [
    '人', '自転車', '車', 'バイク', '飛行機', 'バス', '電車', 'トラック', 'ボート',
    '信号機', '消火栓', '停止標識', 'パーキングメーター', 'ベンチ', '鳥', '猫',
    '犬', '馬', '羊', '牛', '象', '熊', 'シマウマ', 'キリン', 'リュック',
    '傘', 'ハンドバッグ', 'ネクタイ', 'スーツケース', 'フリスビー', 'スキー', 'スノーボード',
    'ボール', '凧', '野球バット', '野球グローブ', 'スケートボード', 'サーフボード',
    'テニスラケット', 'ボトル', 'ワイングラス', 'カップ', 'フォーク', 'ナイフ', 'スプーン',
    'ボウル', 'バナナ', 'リンゴ', 'サンドイッチ', 'オレンジ', 'ブロッコリー', 'ニンジン',
    'ホットドッグ', 'ピザ', 'ドーナツ', 'ケーキ', '椅子', 'ソファ', '観葉植物', 'ベッド',
    'テーブル', 'トイレ', 'テレビ', 'ノートPC', 'マウス', 'リモコン', 'キーボード',
    'スマートフォン', '電子レンジ', 'オーブン', 'トースター', 'シンク', '冷蔵庫', '本',
    '時計', '花瓶', 'ハサミ', 'ぬいぐるみ', 'ドライヤー', '歯ブラシ',
]


class HailoDetector:
    """Hailo-8 上で YOLO HEF を実行する最小ラッパー。"""

    def __init__(self, hef_path: str):
        from hailo_platform import (HEF, ConfigureParams, FormatType, HailoStreamInterface,
                                    InferVStreams, InputVStreamParams, OutputVStreamParams,
                                    VDevice)
        self._hef = HEF(hef_path)
        self._vdevice = VDevice()
        configure_params = ConfigureParams.create_from_hef(
            self._hef, interface=HailoStreamInterface.PCIe)
        self._network_group = self._vdevice.configure(self._hef, configure_params)[0]
        self._network_group_params = self._network_group.create_params()
        self._input_params = InputVStreamParams.make(
            self._network_group, format_type=FormatType.UINT8)
        self._output_params = OutputVStreamParams.make(
            self._network_group, format_type=FormatType.FLOAT32)
        info = self._hef.get_input_vstream_infos()[0]
        self.input_name = info.name
        self.input_height = info.shape[0]
        self.input_width = info.shape[1]

        # パイプラインとアクティベートは毎回作ると高コストなので保持する
        self._stack = ExitStack()
        self._pipeline = self._stack.enter_context(InferVStreams(
            self._network_group, self._input_params, self._output_params))
        self._stack.enter_context(self._network_group.activate(self._network_group_params))

    def infer(self, image: np.ndarray):
        """NHWC uint8 画像 1 枚を推論し、NMS 出力（クラス別の配列）を返す。"""
        results = self._pipeline.infer({self.input_name: np.expand_dims(image, axis=0)})
        return next(iter(results.values()))

    def close(self):
        self._stack.close()
        self._vdevice.release()


class PerceptionNode(Node):
    """LiDAR 主・カメラ/AI HAT+ 補助の前方障害物判定ノード。"""

    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('system_status_topic', '/system_status')
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.8)
        self.declare_parameter('hef_path', '')
        self.declare_parameter('inference_rate', 4.0)
        self.declare_parameter('score_threshold', 0.4)
        self.declare_parameter('camera_hfov_deg', 66.0)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('scan_max_age', 1.0)
        self.declare_parameter('cluster_gap', 0.25)
        self.declare_parameter('danger_distance', 0.3)
        self.declare_parameter('cpu_temp_warn', 70.0)
        self.declare_parameter('cpu_temp_crit', 78.0)
        self.declare_parameter('hailo_temp_warn', 75.0)
        self.declare_parameter('hailo_temp_crit', 85.0)
        # 実効スループット算出用。model_gops は 1 推論あたりの演算量 [GOP]
        # （YOLOv8n 640x640 = 8.7 GOP）、peak_tops は Hailo-8 の公称性能。
        self.declare_parameter('model_gops', 8.7)
        self.declare_parameter('hailo_peak_tops', 26.0)

        self._load_tunables()
        self.add_on_set_parameters_callback(self._on_set_parameters)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )

        self._lock = threading.Lock()
        self._scan_front = None
        self._scan_sectors = None
        self._scan_stamp = 0.0
        self._scan_bearings = []
        self._frame = None
        self._frame_stamp = 0.0
        self._cpu_temp = None
        self._hailo_temp = None
        self._detections = []
        self._detection_stamp = 0.0
        self._inference_ms = None
        self._inference_times = deque(maxlen=30)
        self._thermal_state = 'normal'
        self._thermal_scale = 1.0
        self._detector_note = ''

        self.status_pub = self.create_publisher(
            String, self.get_parameter('obstacle_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, sensor_qos)
        self.create_subscription(
            CompressedImage, self.get_parameter('camera_topic').value,
            self._camera_cb, sensor_qos)
        self.create_subscription(
            String, self.get_parameter('system_status_topic').value, self._system_cb, 10)

        self._detector = None
        self._detector_lock = threading.Lock()
        hef_path = self.get_parameter('hef_path').value
        if hef_path:
            try:
                self._detector = HailoDetector(hef_path)
                self.get_logger().info(f'Hailo 推論を初期化しました: {hef_path}')
            except Exception as exc:  # noqa: BLE001 - 実機依存のため握りつぶして通知する
                self._detector_note = f'Hailo 推論を初期化できません: {exc}'
                self.get_logger().warning(self._detector_note)
        else:
            self._detector_note = 'hef_path が未設定のためカメラ推論は無効です'

        self._infer_busy = False
        self.create_timer(0.2, self._inference_tick)
        self.create_timer(0.2, self._publish_status)

    # --- パラメータ ---
    def _load_tunables(self):
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.camera_hfov = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        # LiDAR の 0° とロボット前方のずれ（取り付け向きの補正）
        self.scan_angle_offset = math.radians(
            float(self.get_parameter('scan_angle_offset_deg').value))
        self.scan_max_age = float(self.get_parameter('scan_max_age').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.cpu_temp_warn = float(self.get_parameter('cpu_temp_warn').value)
        self.cpu_temp_crit = float(self.get_parameter('cpu_temp_crit').value)
        self.hailo_temp_warn = float(self.get_parameter('hailo_temp_warn').value)
        self.hailo_temp_crit = float(self.get_parameter('hailo_temp_crit').value)
        self.model_gops = float(self.get_parameter('model_gops').value)
        self.hailo_peak_tops = float(self.get_parameter('hailo_peak_tops').value)

    def _on_set_parameters(self, params):
        """距離・推論レート・温度閾値を実行中に変更できるようにする。"""
        attrs = {
            'front_angle_deg': lambda v: setattr(self, 'front_angle', math.radians(float(v))),
            'stop_distance': lambda v: setattr(self, 'stop_distance', float(v)),
            'slow_distance': lambda v: setattr(self, 'slow_distance', float(v)),
            'inference_rate': lambda v: setattr(self, 'inference_rate', float(v)),
            'score_threshold': lambda v: setattr(self, 'score_threshold', float(v)),
            'camera_hfov_deg': lambda v: setattr(self, 'camera_hfov', math.radians(float(v))),
            'scan_angle_offset_deg': lambda v: setattr(
                self, 'scan_angle_offset', math.radians(float(v))),
            'scan_max_age': lambda v: setattr(self, 'scan_max_age', float(v)),
            'cluster_gap': lambda v: setattr(self, 'cluster_gap', float(v)),
            'danger_distance': lambda v: setattr(self, 'danger_distance', float(v)),
            'cpu_temp_warn': lambda v: setattr(self, 'cpu_temp_warn', float(v)),
            'cpu_temp_crit': lambda v: setattr(self, 'cpu_temp_crit', float(v)),
            'model_gops': lambda v: setattr(self, 'model_gops', float(v)),
            'hailo_peak_tops': lambda v: setattr(self, 'hailo_peak_tops', float(v)),
            'hailo_temp_warn': lambda v: setattr(self, 'hailo_temp_warn', float(v)),
            'hailo_temp_crit': lambda v: setattr(self, 'hailo_temp_crit', float(v)),
        }
        for param in params:
            apply = attrs.get(param.name)
            if apply is not None:
                apply(param.value)
        return SetParametersResult(successful=True)

    # --- サブスクライバ ---
    def _scan_cb(self, msg: LaserScan):
        half = self.front_angle / 2.0
        front = []
        sectors = {'left': [], 'front': [], 'right': []}
        bearings = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r > msg.range_max:
                continue
            angle = self._normalize(
                msg.angle_min + i * msg.angle_increment + self.scan_angle_offset)
            if abs(angle) <= math.pi / 2.0:
                bearings.append((angle, r))
            if abs(angle) <= half:
                front.append(r)
                if angle > half / 3.0:
                    sectors['left'].append(r)
                elif angle < -half / 3.0:
                    sectors['right'].append(r)
                else:
                    sectors['front'].append(r)
        with self._lock:
            self._scan_front = min(front) if front else None
            self._scan_sectors = {
                k: (round(min(v), 3) if v else None) for k, v in sectors.items()}
            self._scan_bearings = bearings
            self._scan_stamp = time.time()

    def _camera_cb(self, msg: CompressedImage):
        with self._lock:
            self._frame = bytes(msg.data)
            self._frame_stamp = time.time()

    def _system_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        cpu = (data.get('cpu') or {}).get('temperature_c')
        hailo = (data.get('hailo') or {}).get('temperature_c')
        with self._lock:
            self._cpu_temp = cpu
            self._hailo_temp = hailo

    def _distance_for_box(self, x_min: float, x_max: float):
        """画像上の横位置を方位角に変換し、LiDAR から距離を引く。

        画像左端が +HFOV/2、右端が -HFOV/2（ROS の左旋回正）に対応する。
        方位窓内の点は距離でクラスタリングし、最も手前のまとまった面の
        代表距離（中央値）を返す。単発の外れ点で極端に近い値にならない。
        """
        half = self.camera_hfov / 2.0
        angle_max = (0.5 - x_min) * self.camera_hfov
        angle_min = (0.5 - x_max) * self.camera_hfov
        angle_min = max(-half, angle_min)
        angle_max = min(half, angle_max)
        with self._lock:
            bearings = self._scan_bearings
            scan_stamp = self._scan_stamp
        if time.time() - scan_stamp > self.scan_max_age:
            return None
        hits = sorted(r for angle, r in bearings if angle_min <= angle <= angle_max)
        if not hits:
            return None
        cluster = [hits[0]]
        clusters = [cluster]
        for r in hits[1:]:
            if r - cluster[-1] > self.cluster_gap:
                cluster = [r]
                clusters.append(cluster)
            else:
                cluster.append(r)
        min_size = 2 if len(hits) >= 4 else 1
        for cluster in clusters:
            if len(cluster) >= min_size:
                return round(cluster[len(cluster) // 2], 3)
        return round(hits[0], 3)

    # --- サーマルガバナ ---
    def _update_thermal(self):
        with self._lock:
            cpu = self._cpu_temp
            hailo = self._hailo_temp
        state, scale = 'normal', 1.0
        if (cpu is not None and cpu >= self.cpu_temp_crit) or \
                (hailo is not None and hailo >= self.hailo_temp_crit):
            state, scale = 'critical', 0.0
        elif (cpu is not None and cpu >= self.cpu_temp_warn) or \
                (hailo is not None and hailo >= self.hailo_temp_warn):
            state, scale = 'warn', 0.4
        with self._lock:
            self._thermal_state = state
            self._thermal_scale = scale
        return scale

    # --- 推論 ---
    def _inference_tick(self):
        scale = self._update_thermal()
        if self._detector is None or self._infer_busy or scale <= 0.0:
            return
        interval = 1.0 / max(0.2, self.inference_rate * scale)
        with self._lock:
            frame = self._frame
            frame_stamp = self._frame_stamp
            last = self._detection_stamp
        if frame is None or time.time() - last < interval:
            return
        if time.time() - frame_stamp > 2.0:
            return
        self._infer_busy = True
        threading.Thread(target=self._run_inference, args=(frame,), daemon=True).start()

    def _run_inference(self, frame: bytes):
        try:
            import cv2
            image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return
            resized = cv2.resize(
                image, (self._detector.input_width, self._detector.input_height))
            rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            start = time.time()
            with self._detector_lock:
                raw = self._detector.infer(rgb)
            elapsed_ms = (time.time() - start) * 1000.0
            detections = self._parse_detections(raw)
            with self._lock:
                self._detections = detections
                self._detection_stamp = time.time()
                self._inference_ms = round(elapsed_ms, 1)
                self._inference_times.append(self._detection_stamp)
        except Exception as exc:  # noqa: BLE001 - 推論失敗でノードを落とさない
            self._detector_note = f'推論エラー: {exc}'
            self.get_logger().warning(self._detector_note)
        finally:
            self._infer_busy = False

    def _parse_detections(self, raw):
        """HAILO NMS BY CLASS 出力（正規化座標）を検出リストへ変換する。"""
        per_class = raw[0] if isinstance(raw, (list, tuple)) or (
            isinstance(raw, np.ndarray) and raw.dtype == object) else raw
        detections = []
        for class_id, boxes in enumerate(per_class):
            if boxes is None or len(boxes) == 0:
                continue
            for box in boxes:
                score = float(box[4])
                if score < self.score_threshold:
                    continue
                y_min, x_min, y_max, x_max = (float(v) for v in box[:4])
                cx = (x_min + x_max) / 2.0
                distance = self._distance_for_box(x_min, x_max)
                detections.append({
                    'distance': distance,
                    'danger': distance is not None and distance <= self.danger_distance,
                    'label': COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES)
                    else str(class_id),
                    'score': round(score, 3),
                    'box': [round(x_min, 3), round(y_min, 3), round(x_max, 3), round(y_max, 3)],
                    'center_x': round(cx, 3),
                    'area': round(max(0.0, x_max - x_min) * max(0.0, y_max - y_min), 4),
                })
        detections.sort(key=lambda d: d['area'], reverse=True)
        return detections[:10]

    # --- 判定結果の配信 ---
    def _publish_status(self):
        now = time.time()
        with self._lock:
            front = self._scan_front
            sectors = self._scan_sectors
            scan_age = now - self._scan_stamp if self._scan_stamp else None
            detections = list(self._detections)
            det_age = now - self._detection_stamp if self._detection_stamp else None
            thermal_state = self._thermal_state
            thermal_scale = self._thermal_scale
            cpu_temp = self._cpu_temp
            hailo_temp = self._hailo_temp
            inference_ms = self._inference_ms
            stamps = list(self._inference_times)

        scan_valid = front is not None and scan_age is not None and scan_age < 1.5
        if not scan_valid:
            level, reason = 'unknown', 'LiDAR データなし'
        elif front <= self.stop_distance:
            level, reason = 'stop', f'前方 {front:.2f} m に障害物'
        elif front <= self.slow_distance:
            level, reason = 'slow', f'前方 {front:.2f} m に接近'
        else:
            level, reason = 'clear', '前方に障害物なし'

        if det_age is not None and det_age > 3.0:
            detections = []

        # 前方セクターに写っている物体のみ障害物種別として扱う
        labels = [
            f"{d['label']} {d['distance']:.2f}m" if d['distance'] is not None else d['label']
            for d in detections if 0.25 <= d['center_x'] <= 0.75
        ]

        payload = {
            'stamp': round(now, 3),
            'level': level,
            'reason': reason,
            'front_distance': round(front, 3) if front is not None else None,
            'sectors': sectors,
            'stop_distance': self.stop_distance,
            'slow_distance': self.slow_distance,
            'danger_distance': self.danger_distance,
            'front_angle_deg': round(math.degrees(self.front_angle), 1),
            'camera_hfov_deg': round(math.degrees(self.camera_hfov), 1),
            'speed_scale': 0.0 if level == 'stop' else (0.5 if level == 'slow' else 1.0),
            'detections': detections,
            'front_labels': labels,
            'inference_ms': inference_ms,
            'inference_age': round(det_age, 2) if det_age is not None else None,
            'throughput': self._throughput(stamps, inference_ms, now),
            'thermal': {
                'state': thermal_state,
                'inference_scale': thermal_scale,
                'cpu_temp_c': cpu_temp,
                'hailo_temp_c': hailo_temp,
            },
            'note': self._detector_note,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)

    def _throughput(self, stamps, inference_ms, now):
        """実測推論レートから実効スループットを換算する。

        `now_tops` は実際に回しているレートでの演算量、`max_tops` は同じ推論を
        隙間なく連続実行した場合の演算量（= 現状の実力）。Hailo は実効 TOPS を
        直接報告しないため、モデルの演算量×レートによる推定値。
        """
        recent = [t for t in stamps if now - t <= 10.0]
        fps = None
        if len(recent) >= 2:
            span = recent[-1] - recent[0]
            if span > 0:
                fps = (len(recent) - 1) / span
        max_fps = 1000.0 / inference_ms if inference_ms else None
        gops = self.model_gops
        now_tops = fps * gops / 1000.0 if fps else None
        max_tops = max_fps * gops / 1000.0 if max_fps else None
        return {
            'fps': round(fps, 2) if fps else None,
            'max_fps': round(max_fps, 1) if max_fps else None,
            'now_tops': round(now_tops, 3) if now_tops else None,
            'max_tops': round(max_tops, 2) if max_tops else None,
            'peak_tops': self.hailo_peak_tops,
            'utilization': round(max_tops / self.hailo_peak_tops, 3)
            if max_tops and self.hailo_peak_tops else None,
            'model_gops': gops,
        }

    @staticmethod
    def _normalize(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._detector is not None:
            node._detector.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
