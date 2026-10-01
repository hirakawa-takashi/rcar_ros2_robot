"""カメラの正面に置いた目印から、LiDAR の 0° と車体前方のずれを求める。

車体の前方は「カメラの光軸の向き」と決める。カメラの画面の真ん中（左右）に
目印を 1 つ置き、LiDAR のいちばん近い物をその目印とみなして、
目印がカメラの光軸の上に来るように scan_angle_offset_deg を決める。
"""

import math
import os

import yaml

ALIGN_NODES = ('dashboard_node', 'perception_node', 'autonomy_node')
DEFAULT_CALIBRATION_FILE = os.path.expanduser('~/.config/ai_car/lidar_calibration.yaml')


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def scan_clusters(ranges, angle_min, angle_increment, range_min, range_max,
                  near=0.25, far=1.5, gap=0.05):
    """near〜far の点を、隣どうしの距離差が gap 以内のまとまりに分ける。

    戻り値: [(中心の生角度 [rad], 距離の中央値 [m], 点数), ...]
    """
    n = len(ranges)
    ok = [math.isfinite(r) and range_min < r <= range_max and near <= r <= far
          for r in ranges]
    if not any(ok):
        return []
    # 360° 一周しているので、空きの所から始めて端をまたぐまとまりを 1 つにする
    start = next((i for i in range(n) if not ok[i]), 0)
    groups, cur, prev = [], [], None
    for k in range(n):
        i = (start + k) % n
        if not ok[i]:
            if cur:
                groups.append(cur)
            cur, prev = [], None
            continue
        if prev is not None and abs(ranges[i] - prev) > gap:
            groups.append(cur)
            cur = []
        cur.append(k)
        prev = ranges[i]
    if cur:
        groups.append(cur)
    out = []
    for g in groups:
        rs = sorted(ranges[(start + k) % n] for k in g)
        mid = (g[0] + g[-1]) / 2.0
        angle = wrap(angle_min + ((start + mid) % n) * angle_increment)
        out.append((angle, rs[len(rs) // 2], len(g)))
    return out


def pick_target(clusters, min_points=3, margin=0.15, same_object_deg=15.0):
    """いちばん近いまとまりを目印にする。

    目印から same_object_deg 以内のまとまりは同じ物の一部とみなす。それ以外で
    目印との距離の差が margin より小さい物があれば、どれが目印か決められないので None。
    """
    cands = sorted((c for c in clusters if c[2] >= min_points), key=lambda c: c[1])
    if not cands:
        return None, 'LiDAR に 0.25〜1.5 m の物が見えない'
    same = math.radians(same_object_deg)
    cands = [cands[0]] + [c for c in cands[1:] if abs(wrap(c[0] - cands[0][0])) > same]
    if len(cands) > 1 and cands[1][1] - cands[0][1] < margin:
        return None, (f'目印（{cands[0][1]:.2f} m）と同じくらい近い物がほかにもある'
                      f'（{cands[1][1]:.2f} m）')
    return cands[0], ''


def offset_for_target(raw_angle, r, bearing, cam_x, cam_y):
    """目印がカメラから bearing の向きに見えるときの scan_angle_offset [rad]。

    cam_x, cam_y: LiDAR の回転中心から見たカメラの位置（x 前・y 左）[m]。
    目印の位置 = カメラ + t·(cos b, sin b)、LiDAR からの距離が r になる t を求める。
    """
    b = cam_x * math.cos(bearing) + cam_y * math.sin(bearing)
    c = cam_x * cam_x + cam_y * cam_y - r * r
    disc = b * b - c
    if disc < 0.0:
        return None
    t = -b + math.sqrt(disc)
    if t <= 0.0:
        return None
    px = cam_x + t * math.cos(bearing)
    py = cam_y + t * math.sin(bearing)
    return wrap(math.atan2(py, px) - raw_angle)


def circular_median_deg(angles_rad):
    """角度のまとまりの真ん中（度）。±180° をまたいでも正しく出す。"""
    if not angles_rad:
        return None
    ref = math.atan2(sum(math.sin(a) for a in angles_rad),
                     sum(math.cos(a) for a in angles_rad))
    rel = sorted(wrap(a - ref) for a in angles_rad)
    return math.degrees(wrap(ref + rel[len(rel) // 2]))


def load_offset_deg(path):
    """保存した補正値を読む。ファイルが無い・壊れているときは None。"""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return float(data['dashboard_node']['ros__parameters']['scan_angle_offset_deg'])
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError):
        return None


def save_offset_deg(path, offset_deg):
    """3 つのノードに同じ値を入れる、ROS 2 のパラメータファイルとして保存する。"""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    value = round(float(offset_deg), 2)
    data = {name: {'ros__parameters': {'scan_angle_offset_deg': value}}
            for name in ALIGN_NODES}
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        f.write('# ダッシュボードの「カメラの正面で前方を合わせる」で書いたファイル\n')
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path)
