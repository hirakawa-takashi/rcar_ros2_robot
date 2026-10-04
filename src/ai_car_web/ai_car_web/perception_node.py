"""AI-CAR 前方障害物判定ノード。

LiDAR (`/scan`) を主情報として前方セクターの障害物距離と危険度を判定し、
カメラ (`/camera/image_raw/compressed`) の画像を AI HAT+ (Hailo-8) の
YOLO 推論にかけて障害物の種別を補足する。
CPU / AI HAT+ の温度に応じて推論レートを自動的に落とす（サーマルガバナ）。

判定結果は `/obstacle_status`（std_msgs/String, JSON）へ publish する。
"""

import json
import math
import statistics
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


LEVEL_RANK = {'unknown': 0, 'clear': 0, 'slow': 1, 'stop': 2}


CAMERA_ASPECT = 540.0 / 960.0


def image_x_to_bearing(x, hfov):
    """画像の横位置（0 = 左端、1 = 右端）を、カメラから見た方位角 [rad]（左が正）にする（ピンホール）。"""
    return math.atan((0.5 - x) * 2.0 * math.tan(hfov / 2.0))


def bearing_to_image_x(bearing, hfov):
    """image_x_to_bearing の逆。方位角 [rad]（左が正）を画像の横位置（0 = 左端、1 = 右端）にする。"""
    return 0.5 - math.tan(bearing) / (2.0 * math.tan(hfov / 2.0))


def scan_point_in_camera(r, angle, offset_x, offset_y):
    """LiDAR の点（距離 r、方位 angle）を、カメラから見た方位角と前方の距離にする。

    offset_x / offset_y は LiDAR の回転中心から見たカメラのレンズの位置（x: 前、y: 左）。
    カメラより後ろの点は None。
    """
    cx = r * math.cos(angle) - offset_x
    cy = r * math.sin(angle) - offset_y
    if cx <= 0.0:
        return None
    return math.atan2(cy, cx), cx


def _iou(a, b):
    """検出枠または [x_min, y_min, x_max, y_max] 同士の IoU を返す。"""
    box_a = a['box'] if isinstance(a, dict) else a
    box_b = b['box'] if isinstance(b, dict) else b
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_w = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    inter_h = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = inter_w * inter_h
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - intersection
    return intersection / union if union else 0.0


def _confirm_detections(history, confirm):
    """履歴中の一定数以上で同じ物体が確認された最新検出だけを返す。"""
    latest = list(history[-1]) if history else []
    if confirm <= 1:
        return latest
    confirmed = []
    for detection in latest:
        matches = sum(
            any(
                candidate.get('label') == detection.get('label')
                and _iou(candidate, detection) >= 0.3
                for candidate in frame
            )
            for frame in history
        )
        if matches >= confirm:
            confirmed.append(detection)
    return confirmed


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
        # Jetson（NanoOWL）の物体検出。LiDAR の距離を付けて表示するだけで、level / speed_scale は変えない
        self.declare_parameter('jetson_topic', '/jetson_detections')
        self.declare_parameter('jetson_max_age', 2.0)
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.8)
        self.declare_parameter('hef_path', '')
        self.declare_parameter('inference_rate', 4.0)
        self.declare_parameter('score_threshold', 0.5)
        self.declare_parameter('detect_confirm_frames', 2)
        self.declare_parameter('detect_history_frames', 3)
        self.declare_parameter('camera_hfov_deg', 66.0)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        # LiDAR の回転中心から見たカメラのレンズの位置 [m]（x: 前、y: 左）
        self.declare_parameter('camera_offset_x_m', 0.0)
        self.declare_parameter('camera_offset_y_m', 0.0)
        self.declare_parameter('scan_max_age', 1.0)
        self.declare_parameter('cluster_gap', 0.25)
        self.declare_parameter('danger_distance', 0.3)
        # 人・動物（living_labels）を見つけたときの早めの減速・停止。
        # living_guard が false の間は判定を表示するだけで、level / speed_scale は変えない
        self.declare_parameter('living_guard', False)
        self.declare_parameter('living_labels', ['人', '犬', '猫', '鳥'])
        self.declare_parameter('living_slow_distance', 1.5)
        self.declare_parameter('living_stop_distance', 0.6)
        # 2 眼カメラの距離（stereo_depth_node）で見つけた通り道の中の物。
        # LiDAR の同じ方向の距離が stereo_low_margin 以上遠い（光が上を通る）ものだけを低い物とする。
        # stereo_guard が false の間は判定を表示するだけ（dashboard.yaml は true）
        self.declare_parameter('stereo_topic', '/stereo/depth_status')
        self.declare_parameter('stereo_guard', False)
        self.declare_parameter('stereo_slow_distance', 0.8)
        self.declare_parameter('stereo_stop_distance', 0.45)
        self.declare_parameter('stereo_low_margin', 0.15)
        self.declare_parameter('stereo_max_age', 1.5)
        # 物の枠の距離: 2 眼のマスの距離（/stereo/depth_grid）で測れればそれを使い、
        # 測れない所（左はしなど）は LiDAR の距離にする。空で 2 眼を使わない
        self.declare_parameter('stereo_grid_topic', '/stereo/depth_grid')
        self.declare_parameter('stereo_grid_min_ratio', 0.3)
        self.declare_parameter('cpu_temp_warn', 70.0)
        self.declare_parameter('cpu_temp_crit', 78.0)
        self.declare_parameter('hailo_temp_warn', 75.0)
        self.declare_parameter('hailo_temp_crit', 85.0)
        # 実効スループット算出用。model_gops は 1 推論あたりの演算量 [GOP]
        # （YOLOv8m 640x640 = 78.9 GOP）、peak_tops は Hailo-8 の公称性能。
        self.declare_parameter('model_gops', 28.6)
        self.declare_parameter('hailo_peak_tops', 26.0)

        self._load_tunables()
        self._detection_history = deque(maxlen=self.detect_history_frames)
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
        self._inference_ms_recent = deque(maxlen=30)
        self._thermal_state = 'normal'
        self._thermal_scale = 1.0
        self._detector_note = ''
        self._stereo = None
        self._stereo_stamp = 0.0
        self._grid = None
        self._grid_stamp = 0.0
        self._jetson = None
        self._jetson_stamp = 0.0

        self.status_pub = self.create_publisher(
            String, self.get_parameter('obstacle_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, sensor_qos)
        self.create_subscription(
            CompressedImage, self.get_parameter('camera_topic').value,
            self._camera_cb, sensor_qos)
        self.create_subscription(
            String, self.get_parameter('system_status_topic').value, self._system_cb, 10)
        stereo_topic = self.get_parameter('stereo_topic').value
        if stereo_topic:
            self.create_subscription(String, stereo_topic, self._stereo_cb, 10)
        grid_topic = self.get_parameter('stereo_grid_topic').value
        if grid_topic:
            self.create_subscription(String, grid_topic, self._grid_cb, 10)
        jetson_topic = self.get_parameter('jetson_topic').value
        if jetson_topic:
            self.create_subscription(String, jetson_topic, self._jetson_cb, 10)

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
        self._infer_started = 0.0
        self._infer_frame_stamp = 0.0
        self.create_timer(0.02, self._inference_tick)
        self.create_timer(0.2, self._publish_status)

    # --- パラメータ ---
    def _load_tunables(self):
        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.slow_distance = float(self.get_parameter('slow_distance').value)
        self.inference_rate = float(self.get_parameter('inference_rate').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.detect_confirm_frames = max(
            1, int(self.get_parameter('detect_confirm_frames').value))
        self.detect_history_frames = max(
            1, int(self.get_parameter('detect_history_frames').value))
        self.camera_hfov = math.radians(float(self.get_parameter('camera_hfov_deg').value))
        # LiDAR の 0° とロボット前方のずれ（取り付け向きの補正）
        self.scan_angle_offset = math.radians(
            float(self.get_parameter('scan_angle_offset_deg').value))
        self.camera_offset_x = float(self.get_parameter('camera_offset_x_m').value)
        self.camera_offset_y = float(self.get_parameter('camera_offset_y_m').value)
        self.scan_max_age = float(self.get_parameter('scan_max_age').value)
        self.cluster_gap = float(self.get_parameter('cluster_gap').value)
        self.danger_distance = float(self.get_parameter('danger_distance').value)
        self.living_guard = bool(self.get_parameter('living_guard').value)
        self.living_labels = set(self.get_parameter('living_labels').value)
        self.living_slow_distance = float(self.get_parameter('living_slow_distance').value)
        self.living_stop_distance = float(self.get_parameter('living_stop_distance').value)
        self.stereo_guard = bool(self.get_parameter('stereo_guard').value)
        self.stereo_slow_distance = float(self.get_parameter('stereo_slow_distance').value)
        self.stereo_stop_distance = float(self.get_parameter('stereo_stop_distance').value)
        self.stereo_low_margin = float(self.get_parameter('stereo_low_margin').value)
        self.stereo_max_age = float(self.get_parameter('stereo_max_age').value)
        self.stereo_grid_min_ratio = float(self.get_parameter('stereo_grid_min_ratio').value)
        self.jetson_max_age = float(self.get_parameter('jetson_max_age').value)
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
            'detect_confirm_frames': lambda v: setattr(
                self, 'detect_confirm_frames', max(1, int(v))),
            'detect_history_frames': self._set_detection_history_frames,
            'camera_hfov_deg': lambda v: setattr(self, 'camera_hfov', math.radians(float(v))),
            'scan_angle_offset_deg': lambda v: setattr(
                self, 'scan_angle_offset', math.radians(float(v))),
            'camera_offset_x_m': lambda v: setattr(self, 'camera_offset_x', float(v)),
            'camera_offset_y_m': lambda v: setattr(self, 'camera_offset_y', float(v)),
            'scan_max_age': lambda v: setattr(self, 'scan_max_age', float(v)),
            'cluster_gap': lambda v: setattr(self, 'cluster_gap', float(v)),
            'danger_distance': lambda v: setattr(self, 'danger_distance', float(v)),
            'living_guard': lambda v: setattr(self, 'living_guard', bool(v)),
            'living_labels': lambda v: setattr(self, 'living_labels', set(v)),
            'living_slow_distance': lambda v: setattr(self, 'living_slow_distance', float(v)),
            'living_stop_distance': lambda v: setattr(self, 'living_stop_distance', float(v)),
            'stereo_guard': lambda v: setattr(self, 'stereo_guard', bool(v)),
            'stereo_slow_distance': lambda v: setattr(self, 'stereo_slow_distance', float(v)),
            'stereo_stop_distance': lambda v: setattr(self, 'stereo_stop_distance', float(v)),
            'stereo_low_margin': lambda v: setattr(self, 'stereo_low_margin', float(v)),
            'stereo_max_age': lambda v: setattr(self, 'stereo_max_age', float(v)),
            'stereo_grid_min_ratio': lambda v: setattr(self, 'stereo_grid_min_ratio', float(v)),
            'jetson_max_age': lambda v: setattr(self, 'jetson_max_age', float(v)),
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

    def _set_detection_history_frames(self, value):
        history_frames = max(1, int(value))
        if history_frames == self.detect_history_frames:
            return
        self.detect_history_frames = history_frames
        self._detection_history = deque(self._detection_history, maxlen=history_frames)

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
            point = scan_point_in_camera(r, angle, self.camera_offset_x, self.camera_offset_y)
            if point is not None:
                bearings.append(point)
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

    def _stereo_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(data, dict):
            with self._lock:
                self._stereo = data
                self._stereo_stamp = time.time()

    def _grid_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            cols, rows = int(data['cols']), int(data['rows'])
            cells = np.asarray(data['cm'], dtype=np.float64).reshape(rows, cols) / 100.0
        except (json.JSONDecodeError, KeyError, TypeError, ValueError):
            return
        with self._lock:
            self._grid = cells
            self._grid_stamp = time.time()

    def _jetson_cb(self, msg: String):
        """Jetson の検出に、同じ方向の LiDAR の距離を付けて保持する。"""
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(data, dict):
            return
        detections = []
        for d in data.get('detections') or []:
            try:
                x_min, y_min, x_max, y_max = (float(v) for v in d['box'])
                label = str(d['label'])
                score = float(d['score'])
            except (KeyError, TypeError, ValueError):
                continue
            distance, source = self._detection_distance(x_min, y_min, x_max, y_max)
            detections.append({
                'label': label,
                'prompt': str(d.get('prompt', '')),
                'score': round(score, 3),
                'box': [round(x_min, 3), round(y_min, 3), round(x_max, 3), round(y_max, 3)],
                'center_x': round((x_min + x_max) / 2.0, 3),
                'distance': distance,
                'distance_source': source,
            })
        with self._lock:
            self._jetson = {
                'model': data.get('model', ''),
                'inference_ms': data.get('inference_ms'),
                'detections': detections,
            }
            self._jetson_stamp = time.time()

    def _jetson_assessment(self, now):
        with self._lock:
            jetson = self._jetson
            age = now - self._jetson_stamp if self._jetson_stamp else None
        alive = jetson is not None and age is not None and age <= self.jetson_max_age
        return {
            'alive': alive,
            'age': round(age, 2) if age is not None else None,
            'model': jetson['model'] if jetson else '',
            'inference_ms': jetson['inference_ms'] if jetson else None,
            'detections': jetson['detections'] if alive else [],
        }

    def _detection_distance(self, x_min, y_min, x_max, y_max):
        """物の枠の距離と、出した方法（'stereo' / 'lidar' / None）。"""
        distance = self._stereo_distance_for_box(x_min, y_min, x_max, y_max)
        if distance is not None:
            return distance, 'stereo'
        distance = self._distance_for_box(x_min, x_max)
        return distance, ('lidar' if distance is not None else None)

    def _stereo_distance_for_box(self, x_min, y_min, x_max, y_max):
        """枠の真ん中（縦横の半分）にかかる 2 眼のマスの距離。

        測れたマスが stereo_grid_min_ratio より少なければ None。マスの距離を近い順に
        まとめ、測れたマスの 3 割以上がある、いちばん手前のまとまりの中央値を返す。
        """
        with self._lock:
            grid = self._grid
            stamp = self._grid_stamp
        if grid is None or time.time() - stamp > self.stereo_max_age:
            return None
        rows, cols = grid.shape
        mx, my = (x_max - x_min) / 4.0, (y_max - y_min) / 4.0
        xs = (np.arange(cols) + 0.5) / cols
        ys = (np.arange(rows) + 0.5) / rows
        cx = np.nonzero((xs >= x_min + mx) & (xs <= x_max - mx))[0]
        cy = np.nonzero((ys >= y_min + my) & (ys <= y_max - my))[0]
        if not len(cx):
            cx = [min(int((x_min + x_max) / 2.0 * cols), cols - 1)]
        if not len(cy):
            cy = [min(int((y_min + y_max) / 2.0 * rows), rows - 1)]
        cells = grid[np.ix_(cy, cx)].ravel()
        vals = np.sort(cells[cells > 0])
        if not len(vals) or len(vals) < self.stereo_grid_min_ratio * len(cells):
            return None
        clusters = [[vals[0]]]
        for r in vals[1:]:
            if r - clusters[-1][-1] > max(self.cluster_gap, 0.1 * r):
                clusters.append([r])
            else:
                clusters[-1].append(r)
        need = max(1, int(math.ceil(0.3 * len(vals))))
        cluster = next((c for c in clusters if len(c) >= need), max(clusters, key=len))
        return round(float(cluster[len(cluster) // 2]), 3)

    def _distance_for_box(self, x_min: float, x_max: float):
        """画像上の横位置を方位角に変換し、LiDAR から距離を引く。

        画像左端が +HFOV/2、右端が -HFOV/2（ROS の左旋回正）に対応する。
        LiDAR の点はカメラの位置（camera_offset_x_m / _y_m）に移し、カメラから見た
        方位で照合する。返す距離はカメラのレンズから前方への距離。
        方位窓内の点は距離でクラスタリングし、最も手前のまとまった面の
        代表距離（中央値）を返す。単発の外れ点で極端に近い値にならない。
        """
        return self._distance_for_bearings(
            image_x_to_bearing(x_max, self.camera_hfov),
            image_x_to_bearing(x_min, self.camera_hfov))

    def _distance_for_bearings(self, angle_min: float, angle_max: float):
        """カメラから見た方位の範囲 [rad]（左が正）の LiDAR の距離。_distance_for_box と同じ求め方。"""
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
        now = time.time()
        # 間隔は前の推論の始まりから数える。同じコマは 2 回推論しない
        if frame is None or now - self._infer_started < interval:
            return
        if frame_stamp == self._infer_frame_stamp or now - frame_stamp > 2.0:
            return
        self._infer_busy = True
        self._infer_started = now
        self._infer_frame_stamp = frame_stamp
        threading.Thread(target=self._run_inference, args=(frame,), daemon=True).start()

    def _run_inference(self, frame: bytes):
        try:
            import cv2
            image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None:
                return
            model_w = self._detector.input_width
            model_h = self._detector.input_height
            img_h, img_w = image.shape[:2]
            r = min(model_w / img_w, model_h / img_h)
            resized_w = round(img_w * r)
            resized_h = round(img_h * r)
            resized = cv2.resize(image, (resized_w, resized_h))
            pad_x = (model_w - resized_w) // 2
            pad_y = (model_h - resized_h) // 2
            letterboxed = np.full((model_h, model_w, 3), 114, dtype=np.uint8)
            letterboxed[pad_y:pad_y + resized_h, pad_x:pad_x + resized_w] = resized
            rgb = cv2.cvtColor(letterboxed, cv2.COLOR_BGR2RGB)
            start = time.time()
            with self._detector_lock:
                raw = self._detector.infer(rgb)
            elapsed_ms = (time.time() - start) * 1000.0
            geom = (r, pad_x, pad_y, img_w, img_h)
            detections = self._parse_detections(raw, geom)
            self._detection_history.append(detections)
            detections = _confirm_detections(
                self._detection_history, self.detect_confirm_frames)
            detections.sort(key=lambda d: d['area'], reverse=True)
            with self._lock:
                self._detections = detections[:10]
                self._detection_stamp = time.time()
                self._inference_ms = round(elapsed_ms, 1)
                self._inference_times.append(self._detection_stamp)
                self._inference_ms_recent.append(elapsed_ms)
        except Exception as exc:  # noqa: BLE001 - 推論失敗でノードを落とさない
            self._detector_note = f'推論エラー: {exc}'
            self.get_logger().warning(self._detector_note)
        finally:
            self._infer_busy = False

    def _parse_detections(self, raw, geom):
        """HAILO NMS BY CLASS 出力（正規化座標）を検出リストへ変換する。"""
        r, pad_x, pad_y, img_w, img_h = geom
        model_w = self._detector.input_width
        model_h = self._detector.input_height
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
                x_min = (x_min * model_w - pad_x) / (img_w * r)
                x_max = (x_max * model_w - pad_x) / (img_w * r)
                y_min = (y_min * model_h - pad_y) / (img_h * r)
                y_max = (y_max * model_h - pad_y) / (img_h * r)
                x_min, x_max = max(0.0, min(1.0, x_min)), max(0.0, min(1.0, x_max))
                y_min, y_max = max(0.0, min(1.0, y_min)), max(0.0, min(1.0, y_max))
                if x_max <= x_min or y_max <= y_min:
                    continue
                cx = (x_min + x_max) / 2.0
                distance, source = self._detection_distance(x_min, y_min, x_max, y_max)
                detections.append({
                    'distance': distance,
                    'distance_source': source,
                    'danger': distance is not None and distance <= self.danger_distance,
                    'label': COCO_CLASSES[class_id] if class_id < len(COCO_CLASSES)
                    else str(class_id),
                    'score': round(score, 3),
                    'box': [round(x_min, 3), round(y_min, 3), round(x_max, 3), round(y_max, 3)],
                    'center_x': round(cx, 3),
                    'area': round(max(0.0, x_max - x_min) * max(0.0, y_max - y_min), 4),
                })
        return detections

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
            recent_ms = list(self._inference_ms_recent)

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

        living = self._living_assessment(detections)
        if self.living_guard and LEVEL_RANK[living['level']] > LEVEL_RANK.get(level, 0):
            level, reason = living['level'], living['reason']
        stereo = self._stereo_assessment(now)
        if self.stereo_guard and LEVEL_RANK[stereo['level']] > LEVEL_RANK.get(level, 0):
            level, reason = stereo['level'], stereo['reason']

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
            'camera_offset_x_m': self.camera_offset_x,
            'camera_offset_y_m': self.camera_offset_y,
            'speed_scale': 0.0 if level == 'stop' else (0.5 if level == 'slow' else 1.0),
            'detections': detections,
            'front_labels': labels,
            'living': living,
            'stereo': stereo,
            'jetson': self._jetson_assessment(now),
            'inference_ms': inference_ms,
            'inference_age': round(det_age, 2) if det_age is not None else None,
            'throughput': self._throughput(stamps, recent_ms, now),
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

    def _living_assessment(self, detections):
        """前方に写る人・動物のうち最も近いものから、減速・停止の判定を作る。"""
        nearest = None
        for d in detections:
            if d['label'] not in self.living_labels or d['distance'] is None:
                continue
            if not 0.25 <= d['center_x'] <= 0.75:
                continue
            if nearest is None or d['distance'] < nearest['distance']:
                nearest = d
        level, reason = 'clear', ''
        if nearest is not None:
            where = f"{nearest['label']} {nearest['distance']:.2f} m"
            if nearest['distance'] <= self.living_stop_distance:
                level, reason = 'stop', f'前方 {where} で停止'
            elif nearest['distance'] <= self.living_slow_distance:
                level, reason = 'slow', f'前方 {where} で減速'
        return {
            'enabled': self.living_guard,
            'level': level,
            'reason': reason,
            'label': nearest['label'] if nearest else None,
            'distance': nearest['distance'] if nearest else None,
            'slow_distance': self.living_slow_distance,
            'stop_distance': self.living_stop_distance,
        }

    def _stereo_assessment(self, now):
        """2 眼の距離のいちばん近い物を LiDAR と比べ、低い障害物の減速・停止の判定を作る。"""
        with self._lock:
            stereo = self._stereo
            age = now - self._stereo_stamp if self._stereo_stamp else None
        result = {
            'enabled': self.stereo_guard,
            'ready': False,
            'level': 'clear',
            'reason': '',
            'distance': None,
            'lidar_distance': None,
            'low': False,
            'slow_distance': self.stereo_slow_distance,
            'stop_distance': self.stereo_stop_distance,
        }
        if stereo is None or age is None or age > self.stereo_max_age:
            result['reason'] = '/stereo/depth_status 未受信'
            return result
        if not stereo.get('ok'):
            result['reason'] = stereo.get('reason') or '測れない'
            return result
        result['ready'] = True
        result['objects'] = [self._stereo_object(o) for o in (stereo.get('objects') or [])
                             if o.get('bearing_min_deg') is not None]
        nearest = stereo.get('nearest')
        if not nearest or nearest.get('bearing_min_deg') is None:
            return result
        item = self._stereo_object(nearest)
        result.update(item)
        distance, low = item['distance'], item['low']
        if low:
            where = f'低い障害物 {distance:.2f} m（2 眼）'
            if distance <= self.stereo_stop_distance:
                result['level'], result['reason'] = 'stop', f'前方 {where} で停止'
            elif distance <= self.stereo_slow_distance:
                result['level'], result['reason'] = 'slow', f'前方 {where} で減速'
        return result

    def _stereo_object(self, obj):
        """2 眼の物 1 個の距離・横の位置・横幅・同じ向きの LiDAR の距離・低い物か・映像の上の枠。"""
        distance = obj['distance_m']
        b_min, b_max = (math.radians(obj[k]) for k in ('bearing_min_deg', 'bearing_max_deg'))
        lidar = self._distance_for_bearings(b_min, b_max)
        width = obj.get('width_m')
        if width is None:
            width = round(distance * (math.tan(b_max) - math.tan(b_min)), 3)
        item = {'distance': distance, 'lateral': obj.get('lateral_m'), 'width': width,
                'lidar_distance': lidar,
                'low': lidar is None or lidar > distance + self.stereo_low_margin}
        if obj.get('depression_min_deg') is not None:
            # 前方カメラの映像（960x540）の上の位置。ピンホールとして縦の画角は横から出す
            v_tan = math.tan(self.camera_hfov / 2.0) * CAMERA_ASPECT
            x1, x2 = (min(max(bearing_to_image_x(math.radians(b), self.camera_hfov), 0.0), 1.0)
                      for b in (obj['bearing_max_deg'], obj['bearing_min_deg']))
            y1, y2 = (min(max(0.5 + math.tan(math.radians(e)) / (2.0 * v_tan), 0.0), 1.0)
                      for e in (obj['depression_min_deg'], obj['depression_max_deg']))
            item['box'] = [round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3)]
        return item

    def _throughput(self, stamps, recent_ms, now):
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
        # 1 回の推論時間は CPU の混み具合で 30〜100 ms とばらつくので、最近 30 回の中央値を使う
        inference_ms = statistics.median(recent_ms) if recent_ms else None
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
