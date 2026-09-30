#!/usr/bin/env python3
"""カメラの画像から、床の上の低い障害物を見つけるノード。

カメラは床から camera_height_m の高さに、水平から camera_pitch_deg だけ下を向けて
付いている。床の上の点は遠いほど画面の上に映るので、画面の行ごとに床までの距離
（カメラからの前方の水平距離）を前もって計算できる。

1 コマごとに、
  1. 車の通り道（車幅 + 余裕）を画面の上の台形に置きかえ、その中だけを見る
  2. 通り道のいちばん近い帯（ref_max_m 以内）の色から「床の色」を覚える（Lab の平均・ばらつき）
  3. 各列を下から上へ見て、床の色と違う画素が min_rows 行続いた所を物の足元とする
  4. 足元の行を距離に置きかえ、距離の近い列をまとめて幅 min_width_m 以上のものだけ残す
  5. history_frames コマのうち confirm_frames コマで近い距離に見えたときだけ確定する
結果は /floor_obstacle_status（std_msgs/String の JSON）で配信する。
LiDAR との比較（低い物かどうか）と減速・停止の判定は perception_node が行う。
"""

import json
import math
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def row_distances(height, camera_height, pitch, vfov):
    """画面の各行（上から）に映る床までの距離 [m]。床が映らない行は inf。"""
    v = (np.arange(height) + 0.5) / height
    down = np.arctan((v - 0.5) * 2.0 * math.tan(vfov / 2.0)) + pitch
    dist = np.full(height, np.inf)
    ok = down > 1e-3
    dist[ok] = camera_height / np.tan(down[ok])
    return dist


def corridor_mask(width, dists, hfov, center_y, half_width, max_range):
    """通り道（左右 center_y ± half_width、距離 max_range 以内）に入る画素。"""
    u = (np.arange(width) + 0.5) / width
    tan_u = (0.5 - u) * 2.0 * math.tan(hfov / 2.0)
    finite = np.isfinite(dists) & (dists <= max_range)
    safe = np.where(finite, dists, 0.0)
    lateral = safe[:, None] * tan_u[None, :]
    return finite[:, None] & (np.abs(lateral - center_y) <= half_width)


def nonfloor_mask(lab, mean, tol):
    """床の色（mean ± tol、Lab）から外れる画素。"""
    diff = (lab.astype(np.float32) - mean[None, None, :]) / tol[None, None, :]
    return np.sum(diff * diff, axis=2) > 1.0


def find_obstacle(nonfloor, mask, dists, hfov, min_rows, min_width, gap):
    """通り道の中で最も近い物の足元を探す。

    戻り値は {'distance', 'x_min', 'x_max', 'y_foot', 'width_m'}（正規化座標）か None。
    """
    height, width = nonfloor.shape
    foot = np.full(width, -1)
    for col in range(width):
        run = 0
        for row in range(height - 1, -1, -1):
            if not mask[row, col]:
                if run:
                    break
                continue
            if nonfloor[row, col]:
                run += 1
                if run >= min_rows:
                    foot[col] = row + min_rows - 1
                    break
            else:
                run = 0
    col_dist = np.where(foot >= 0, dists[np.clip(foot, 0, height - 1)], np.inf)
    col_width = 2.0 * math.tan(hfov / 2.0) / width
    best = None
    start = None
    for col in range(width + 1):
        edge = col == width or not np.isfinite(col_dist[col]) or (
            start is not None and abs(col_dist[col] - col_dist[col - 1]) > gap)
        if start is not None and edge:
            seg = col_dist[start:col]
            dist = float(np.median(seg))
            width_m = (col - start) * col_width * dist
            if width_m >= min_width and (best is None or dist < best['distance']):
                best = {
                    'distance': round(dist, 3),
                    'x_min': round(start / width, 3),
                    'x_max': round(col / width, 3),
                    'y_foot': round((int(np.max(foot[start:col])) + 1) / height, 3),
                    'width_m': round(width_m, 3),
                }
            start = None
        if col < width and np.isfinite(col_dist[col]) and start is None:
            start = col
    return best


def confirm(history, needed, gap):
    """最新の検出が、履歴の中で needed コマ以上近い距離に見えていれば返す。"""
    latest = history[-1] if history else None
    if latest is None:
        return None
    count = sum(1 for h in history
                if h is not None and abs(h['distance'] - latest['distance']) <= gap)
    return latest if count >= needed else None


class FloorObstacleNode(Node):
    def __init__(self):
        super().__init__('floor_obstacle_node')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('status_topic', '/floor_obstacle_status')
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('camera_height_m', 0.128)
        self.declare_parameter('camera_pitch_deg', 0.0)
        self.declare_parameter('camera_hfov_deg', 66.0)
        self.declare_parameter('camera_vfov_deg', 40.1)
        # 通り道の中心のカメラからの左右のずれ [m]（左が正）と、片側の幅
        self.declare_parameter('corridor_center_y_m', 0.0)
        self.declare_parameter('corridor_half_width_m', 0.15)
        self.declare_parameter('max_range_m', 1.5)
        self.declare_parameter('ref_max_m', 0.45)
        self.declare_parameter('tol_l', 25.0)
        self.declare_parameter('tol_ab', 8.0)
        self.declare_parameter('tol_sigma', 3.0)
        self.declare_parameter('learn_rate', 0.05)
        self.declare_parameter('warmup_frames', 5)
        self.declare_parameter('min_rows', 3)
        self.declare_parameter('min_width_m', 0.03)
        self.declare_parameter('segment_gap_m', 0.15)
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('history_frames', 3)
        g = self.get_parameter
        self.camera_height = float(g('camera_height_m').value)
        self.pitch = math.radians(float(g('camera_pitch_deg').value))
        self.hfov = math.radians(float(g('camera_hfov_deg').value))
        self.vfov = math.radians(float(g('camera_vfov_deg').value))
        self.center_y = float(g('corridor_center_y_m').value)
        self.half_width = float(g('corridor_half_width_m').value)
        self.max_range = float(g('max_range_m').value)
        self.ref_max = float(g('ref_max_m').value)
        self.tol_min = np.array([float(g('tol_l').value), float(g('tol_ab').value),
                                 float(g('tol_ab').value)], dtype=np.float32)
        self.tol_sigma = float(g('tol_sigma').value)
        self.learn_rate = float(g('learn_rate').value)
        self.warmup = int(g('warmup_frames').value)
        self.min_rows = int(g('min_rows').value)
        self.min_width = float(g('min_width_m').value)
        self.segment_gap = float(g('segment_gap_m').value)
        self.confirm_frames = int(g('confirm_frames').value)
        self.history = deque(maxlen=max(1, int(g('history_frames').value)))

        self._lock = threading.Lock()
        self._frame = None
        self._frame_stamp = 0.0
        self._geometry = None
        self._mean = None
        self._std = None
        self._learned = 0
        self._busy = False
        self._note = ''

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         durability=QoSDurabilityPolicy.VOLATILE, depth=1)
        self.create_subscription(CompressedImage, g('camera_topic').value, self._camera_cb, qos)
        self.pub = self.create_publisher(String, g('status_topic').value, 10)
        self.create_timer(1.0 / max(0.5, float(g('rate').value)), self._tick)

    def _camera_cb(self, msg: CompressedImage):
        with self._lock:
            self._frame = bytes(msg.data)
            self._frame_stamp = time.time()

    def _tick(self):
        if self._busy:
            return
        with self._lock:
            frame, stamp = self._frame, self._frame_stamp
        if frame is None or time.time() - stamp > 2.0:
            self._publish(None, None, None, 'カメラ映像なし')
            return
        self._busy = True
        threading.Thread(target=self._process, args=(frame,), daemon=True).start()

    def _prepare(self, height, width):
        if self._geometry is None or self._geometry[0] != (height, width):
            dists = row_distances(height, self.camera_height, self.pitch, self.vfov)
            mask = corridor_mask(width, dists, self.hfov, self.center_y,
                                 self.half_width, self.max_range)
            ref = mask & (dists <= self.ref_max)[:, None]
            self._geometry = ((height, width), dists, mask, ref)
        return self._geometry[1:]

    def _process(self, frame: bytes):
        start = time.time()
        try:
            import cv2
            image = cv2.imdecode(np.frombuffer(frame, dtype=np.uint8), cv2.IMREAD_REDUCED_COLOR_4)
            if image is None:
                return
            lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.float32)
            dists, mask, ref = self._prepare(*lab.shape[:2])
            if not ref.any():
                self._publish(None, None, dists, '床の見本の帯が画面に入らない（カメラの向きを確認）')
                return
            raw = None
            if self._mean is not None and self._learned >= self.warmup:
                tol = np.maximum(self._std * self.tol_sigma, self.tol_min)
                nonfloor = nonfloor_mask(lab, self._mean, tol)
                raw = find_obstacle(nonfloor, mask, dists, self.hfov, self.min_rows,
                                    self.min_width, self.segment_gap)
            if raw is None or raw['distance'] > self.ref_max + 0.2:
                self._learn(lab[ref])
            self.history.append(raw)
            confirmed = confirm(self.history, self.confirm_frames, self.segment_gap)
            self._publish(raw, confirmed, dists, '', (time.time() - start) * 1000.0)
        except Exception as exc:  # noqa: BLE001 - 処理の失敗でノードを落とさない
            self._note = f'処理エラー: {exc}'
            self.get_logger().warning(self._note)
        finally:
            self._busy = False

    def _learn(self, pixels):
        mean = pixels.mean(axis=0)
        std = pixels.std(axis=0)
        if self._mean is None:
            self._mean, self._std = mean, std
        else:
            a = self.learn_rate
            self._mean = (1 - a) * self._mean + a * mean
            self._std = (1 - a) * self._std + a * std
        self._learned += 1

    def _publish(self, raw, confirmed, dists, note, process_ms=None):
        near = None
        if dists is not None:
            finite = dists[np.isfinite(dists)]
            near = round(float(finite.min()), 3) if finite.size else None
        payload = {
            'stamp': round(time.time(), 3),
            'ready': self._mean is not None and self._learned >= self.warmup,
            'obstacle': confirmed,
            'raw': raw,
            'near_limit_m': near,
            'max_range_m': self.max_range,
            'process_ms': round(process_ms, 1) if process_ms is not None else None,
            'floor_lab': [round(float(v), 1) for v in self._mean] if self._mean is not None else None,
            'note': note or self._note,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FloorObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
