#!/usr/bin/env python3
"""2 眼カメラの左右の画像から距離を出すノード（表示用）。

stereo_camera_node の /stereo/pair_gray（左右を横に並べた白黒）を受け、
calib_file（stereo_calibrate が書いた stereo_calib.yaml）で左右を平行にそろえ、
SGBM で左右のずれ（視差）を求めて距離にする。

距離から 3D の点にして、カメラの高さ camera_height_m と下向きの角度 camera_pitch_deg で
床からの高さを出し、通り道（車幅 + 余裕）の中で床より min_height_m 以上高い点を物とみなす。
いちばん近い物の距離を /stereo/depth_status（std_msgs/String の JSON）に、
距離の色の画像（近い = 赤、遠い = 青、測れない = 黒）を /stereo/depth/compressed に出す。
点を平行化する前の左目の画像（= /camera/image_raw/compressed と同じ見え方）に戻し、
grid_cols x grid_rows のマスごとの前方の距離を /stereo/depth_grid に出す（物の枠の距離に使う）。
走る・止まるの判断には使わない（表示だけ）。
"""

import json
import math
import os
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from ai_car_web.stereo_calib import DEFAULT_DIR


def load_calib(path):
    """stereo_calib.yaml を読む。"""
    with open(path, encoding='utf-8') as f:
        c = yaml.safe_load(f)
    keys = ('K1', 'D1', 'K2', 'D2', 'R1', 'R2', 'P1', 'P2')
    calib = {k: np.array(c[k], dtype=np.float64) for k in keys}
    calib['image_size'] = tuple(int(v) for v in c['image_size'])
    calib['stamp'] = c.get('stamp')
    calib['rms_stereo_px'] = c.get('rms_stereo_px')
    return calib


def rectify_maps(calib, size):
    """size（片目の幅, 高さ）の画像を平行にそろえる表と、焦点距離・中心・レンズの間隔。"""
    scale = size[0] / calib['image_size'][0]
    k1, k2 = calib['K1'].copy(), calib['K2'].copy()
    p1, p2 = calib['P1'].copy(), calib['P2'].copy()
    for m in (k1, k2, p1, p2):
        m[:2] *= scale
    map1 = cv2.initUndistortRectifyMap(k1, calib['D1'], calib['R1'], p1, size, cv2.CV_16SC2)
    map2 = cv2.initUndistortRectifyMap(k2, calib['D2'], calib['R2'], p2, size, cv2.CV_16SC2)
    focal = float(p1[0, 0])
    baseline = float(-p2[0, 3] / p2[0, 0])
    return map1, map2, focal, (float(p1[0, 2]), float(p1[1, 2])), baseline


def pixel_rays(size, focal, center, r1):
    """各画素の向き（z = 1 のときの平行化前の左目の x, y, z）。距離 z を掛けると 3D 点になる。"""
    w, h = size
    v, u = np.mgrid[0:h, 0:w].astype(np.float32)
    rays = np.stack([(u - center[0]) / focal, (v - center[1]) / focal,
                     np.ones_like(u)], axis=-1)
    return rays @ r1.astype(np.float32)


def points_from_disparity(disp, focal, baseline, rays, max_disp):
    """視差 [px] から、左目（平行化する前）の向きの 3D 点 (x 右, y 下, z 前) [m] と有効な画素。

    探す幅のいちばん端（max_disp 以上）の視差は、まちがった対応が多いので使わない。
    """
    valid = (disp > 0.5) & (disp < max_disp)
    z = np.zeros_like(disp)
    z[valid] = focal * baseline / disp[valid]
    return rays * z[..., None], valid


def to_vehicle(pts, camera_height, pitch):
    """カメラの 3D 点を、前 [m]・左 [m]・床からの高さ [m] にする（pitch は下向きが正）。"""
    x, y, z = pts[..., 0], pts[..., 1], pts[..., 2]
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = z * cp - y * sp
    down = z * sp + y * cp
    return forward, -x, camera_height - down


def nearest_obstacle(forward, left, height, valid, center_y, half_width, max_range,
                     min_height, max_height, min_points, camera_height=0.0):
    """通り道の中で床より高い点のうち、いちばん近いまとまり。なければ (None, mask)。"""
    mask = (valid & (forward > 0.1) & (forward <= max_range)
            & (np.abs(left - center_y) <= half_width)
            & (height >= min_height) & (height <= max_height))
    count = int(mask.sum())
    if count < min_points:
        return None, mask
    f = forward[mask]
    near = float(np.percentile(f, 5))
    close = mask & (forward <= near + 0.05)
    cols = np.nonzero(close)[1]
    bearing = np.degrees(np.arctan2(left[close], forward[close]))
    elevation = np.degrees(np.arctan2(camera_height - height[close], forward[close]))
    return {
        'distance_m': round(near, 3),
        'lateral_m': round(float(np.median(left[close])), 3),
        'height_m': round(float(np.median(height[close])), 3),
        'x_min': round(float(cols.min()) / forward.shape[1], 3),
        'x_max': round(float(cols.max() + 1) / forward.shape[1], 3),
        'bearing_min_deg': round(float(bearing.min()), 1),
        'bearing_max_deg': round(float(bearing.max()), 1),
        'depression_min_deg': round(float(elevation.min()), 1),
        'depression_max_deg': round(float(elevation.max()), 1),
        'points': count,
    }, mask


def depth_grid(pts, forward, valid, k1, d1, size, cols, rows, max_range, min_points,
               step=4):
    """マスごとの前方の距離の中央値 [cm]（左上から行ごと、測れないマスは 0）。

    pts は平行化する前の左目の向きの 3D 点なので、K1・D1 で左目の元の画像の位置に戻す。
    """
    sub = (slice(None, None, step), slice(None, None, step))
    f = forward[sub]
    sel = valid[sub] & (f > 0.1) & (f <= max_range)
    cells = np.zeros(cols * rows, dtype=np.int32)
    if not sel.any():
        return cells.tolist()
    p = pts[sub][sel].reshape(-1, 1, 3).astype(np.float64)
    uv, _ = cv2.projectPoints(p, np.zeros(3), np.zeros(3), k1, d1)
    uv = uv.reshape(-1, 2)
    cx = np.floor(uv[:, 0] / size[0] * cols).astype(np.int64)
    cy = np.floor(uv[:, 1] / size[1] * rows).astype(np.int64)
    inside = (cx >= 0) & (cx < cols) & (cy >= 0) & (cy < rows)
    idx = (cy * cols + cx)[inside]
    dist = f[sel][inside]
    order = np.lexsort((dist, idx))
    idx, dist = idx[order], dist[order]
    counts = np.bincount(idx, minlength=cols * rows)
    starts = np.concatenate(([0], np.cumsum(counts)[:-1]))
    ok = counts >= min_points
    cells[ok] = np.round(dist[starts[ok] + counts[ok] // 2] * 100.0).astype(np.int32)
    return cells.tolist()


def colorize(forward, valid, mask, near_m, far_m):
    """距離の色の画像（近い = 赤、遠い = 青、測れない = 黒、通り道の物 = 白で混ぜる）。"""
    t = np.clip((far_m - forward) / max(far_m - near_m, 1e-3), 0.0, 1.0)
    img = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[~valid | (forward <= 0)] = 0
    img[mask] = (img[mask] // 2 + 127).astype(np.uint8)
    return img


class StereoDepthNode(Node):
    """左右の画像から距離を出し、いちばん近い物と距離の色の画像を出す。"""

    def __init__(self):
        super().__init__('stereo_depth_node')
        self.declare_parameter('pair_topic', '/stereo/pair_gray')
        self.declare_parameter('status_topic', '/stereo/depth_status')
        self.declare_parameter('image_topic', '/stereo/depth/compressed')
        self.declare_parameter('grid_topic', '/stereo/depth_grid')
        self.declare_parameter('grid_cols', 32)
        self.declare_parameter('grid_rows', 18)
        self.declare_parameter('grid_max_range_m', 4.0)
        self.declare_parameter('grid_min_points', 6)
        self.declare_parameter('calib_file', os.path.join(DEFAULT_DIR, 'stereo_calib.yaml'))
        self.declare_parameter('num_disparities', 96)
        self.declare_parameter('block_size', 5)
        self.declare_parameter('uniqueness_ratio', 10)
        self.declare_parameter('speckle_window', 100)
        self.declare_parameter('speckle_range', 2)
        self.declare_parameter('camera_height_m', 0.112)
        self.declare_parameter('camera_pitch_deg', 0.0)
        self.declare_parameter('corridor_center_y_m', -0.038)
        self.declare_parameter('corridor_half_width_m', 0.15)
        self.declare_parameter('max_range_m', 2.0)
        self.declare_parameter('min_height_m', 0.04)
        self.declare_parameter('max_height_m', 0.6)
        self.declare_parameter('min_points', 40)
        self.declare_parameter('display_near_m', 0.2)
        self.declare_parameter('display_far_m', 3.0)
        self.declare_parameter('jpeg_quality', 70)
        g = self.get_parameter
        self.calib_file = os.path.expanduser(g('calib_file').value)
        nd = max(16, int(g('num_disparities').value) // 16 * 16)
        bs = int(g('block_size').value) | 1
        self.max_disp = nd - 4
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=nd, blockSize=bs,
            P1=8 * bs * bs, P2=32 * bs * bs, disp12MaxDiff=1,
            uniquenessRatio=int(g('uniqueness_ratio').value),
            speckleWindowSize=int(g('speckle_window').value),
            speckleRange=int(g('speckle_range').value),
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
        self.camera_height = float(g('camera_height_m').value)
        self.pitch = math.radians(float(g('camera_pitch_deg').value))
        self.center_y = float(g('corridor_center_y_m').value)
        self.half_width = float(g('corridor_half_width_m').value)
        self.max_range = float(g('max_range_m').value)
        self.min_height = float(g('min_height_m').value)
        self.max_height = float(g('max_height_m').value)
        self.min_points = int(g('min_points').value)
        self.near_m = float(g('display_near_m').value)
        self.far_m = float(g('display_far_m').value)
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, int(g('jpeg_quality').value)]
        self.grid_cols = max(1, int(g('grid_cols').value))
        self.grid_rows = max(1, int(g('grid_rows').value))
        self.grid_max_range = float(g('grid_max_range_m').value)
        self.grid_min_points = max(1, int(g('grid_min_points').value))

        qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.status_pub = self.create_publisher(String, g('status_topic').value, 10)
        self.image_pub = self.create_publisher(CompressedImage, g('image_topic').value, qos)
        self.grid_pub = self.create_publisher(String, g('grid_topic').value, 10)
        self.create_subscription(Image, g('pair_topic').value, self._pair_cb, qos)

        self._calib = None
        self._calib_mtime = None
        self._maps = None
        self._maps_size = None
        self._rays = None
        self._k1 = None
        self._cond = threading.Condition()
        self._pending = None
        self._times = deque(maxlen=20)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        self.create_timer(1.0, self._idle_status)
        self._last_status = 0.0

    def _pair_cb(self, msg: Image):
        with self._cond:
            self._pending = msg
            self._cond.notify()

    def _publish_status(self, status):
        status['stamp'] = round(time.time(), 3)
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)
        self._last_status = time.monotonic()

    def _idle_status(self):
        if time.monotonic() - self._last_status >= 2.0:
            self._publish_status({'ok': False, 'reason': '左右の画像が届かない', 'nearest': None})

    def _ensure_calib(self, size):
        try:
            mtime = os.path.getmtime(self.calib_file)
        except OSError:
            self._calib = None
            return 'キャリブレーションの値がない（stereo_calibrate で作る）'
        if self._calib is None or mtime != self._calib_mtime:
            try:
                self._calib = load_calib(self.calib_file)
            except (OSError, KeyError, ValueError, yaml.YAMLError) as e:
                self._calib = None
                return f'キャリブレーションの値を読めない: {e}'
            self._calib_mtime = mtime
            self._maps = None
            self.get_logger().info(f'キャリブレーションを読んだ: {self.calib_file}')
        if self._maps is None or self._maps_size != size:
            self._maps = rectify_maps(self._calib, size)
            self._maps_size = size
            self._rays = pixel_rays(size, self._maps[2], self._maps[3], self._calib['R1'])
            self._k1 = self._calib['K1'].copy()
            self._k1[:2] *= size[0] / self._calib['image_size'][0]
        return ''

    def _loop(self):
        while self._running and rclpy.ok():
            with self._cond:
                if self._pending is None:
                    self._cond.wait(timeout=0.5)
                msg, self._pending = self._pending, None
            if msg is None:
                continue
            try:
                self._process(msg)
            except cv2.error as e:
                self.get_logger().warn(f'距離の計算に失敗: {e}')

    def _process(self, msg: Image):
        t0 = time.perf_counter()
        if msg.encoding != 'mono8' or msg.width % 2:
            self._publish_status({'ok': False, 'reason': f'画像の形式が違う: {msg.encoding}',
                                  'nearest': None})
            return
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
        half = msg.width // 2
        size = (half, msg.height)
        reason = self._ensure_calib(size)
        if reason:
            self._publish_status({'ok': False, 'reason': reason, 'nearest': None})
            return
        map1, map2, focal, _, baseline = self._maps
        left = cv2.remap(frame[:, :half], map1[0], map1[1], cv2.INTER_LINEAR)
        right = cv2.remap(frame[:, half:], map2[0], map2[1], cv2.INTER_LINEAR)
        disp = self.sgbm.compute(left, right).astype(np.float32) / 16.0
        pts, valid = points_from_disparity(disp, focal, baseline, self._rays, self.max_disp)
        forward, lateral, height = to_vehicle(pts, self.camera_height, self.pitch)
        nearest, mask = nearest_obstacle(
            forward, lateral, height, valid, self.center_y, self.half_width, self.max_range,
            self.min_height, self.max_height, self.min_points, self.camera_height)
        floor = (valid & (forward > 0.1) & (forward <= self.max_range)
                 & (np.abs(lateral - self.center_y) <= self.half_width)
                 & (np.abs(height) < self.min_height))
        if self.grid_pub.get_subscription_count() > 0:
            grid = String()
            grid.data = json.dumps({
                'cols': self.grid_cols,
                'rows': self.grid_rows,
                'cm': depth_grid(pts, forward, valid, self._k1, self._calib['D1'], size,
                                 self.grid_cols, self.grid_rows, self.grid_max_range,
                                 self.grid_min_points),
                'stamp': round(time.time(), 3),
            })
            self.grid_pub.publish(grid)
        ms = (time.perf_counter() - t0) * 1000.0
        now = time.monotonic()
        self._times.append(now)
        fps = ((len(self._times) - 1) / (self._times[-1] - self._times[0])
               if len(self._times) >= 2 and self._times[-1] > self._times[0] else None)

        if self.image_pub.get_subscription_count() > 0:
            img = colorize(forward, valid, mask, self.near_m, self.far_m)
            if nearest:
                x0 = int(nearest['x_min'] * half)
                x1 = int(nearest['x_max'] * half)
                cv2.rectangle(img, (x0, 0), (x1, msg.height - 1), (255, 255, 255), 2)
            ok, jpeg = cv2.imencode('.jpg', img, self.jpeg_params)
            if ok:
                out = CompressedImage()
                out.header = msg.header
                out.format = 'jpeg'
                out.data = jpeg.tobytes()
                self.image_pub.publish(out)

        self._publish_status({
            'ok': True,
            'reason': '',
            'nearest': nearest,
            'valid_ratio': round(float(valid.mean()), 3),
            'floor_points': int(floor.sum()),
            'floor_height_m': (round(float(np.median(height[floor])), 3)
                               if floor.any() else None),
            'ms': round(ms, 1),
            'fps': round(fps, 1) if fps else None,
            'size': [half, msg.height],
            'baseline_m': round(baseline, 4),
            'calib_stamp': self._calib.get('stamp'),
            'rms_stereo_px': self._calib.get('rms_stereo_px'),
            'max_range_m': self.max_range,
            'corridor_half_width_m': self.half_width,
            'min_height_m': self.min_height,
        })

    def destroy_node(self):
        self._running = False
        with self._cond:
            self._cond.notify()
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoDepthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
