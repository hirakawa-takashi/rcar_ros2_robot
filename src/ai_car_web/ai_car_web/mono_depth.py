"""1 眼カメラの奥行きの AI（SC-Depth v3）の出力から、床から出っぱった低い物を見つける。

AI の奥行きは「何倍か」がわからず（縮尺なし）、遠くほど縮んで出る。そこで距離は AI から
読まない。同じ行（床なら同じ距離）の真ん中の値より手前に見える所を「物」とし、物が床に
ついている一番下の行を、カメラの高さと下向きの角度から床の距離に直す。

LiDAR が届いているときは、LiDAR の点が写る画素で「LiDAR の m × AI の値」を毎回くらべて
倍率を出し（lidar_scale）、AI の奥行きをそのまま m にして物の距離にする。
"""

import math
from collections import deque

import cv2
import numpy as np


def sc_disparity(raw):
    """SC-Depth v3 の出力（sigmoid の前）を、奥行きの逆数に比例する値にする。"""
    return 10.0 / (1.0 + np.exp(-raw)) + 0.01


def camera_rays(rows, cols, hfov_deg, aspect, pitch_deg=0.0):
    """モデルの各行の下向きの tan（床に向く角度）と、各列の左向きの tan。

    カメラはピンホールとし、焦点は横の画角から出す。aspect は元の映像の 高さ / 幅。
    """
    t = math.tan(math.radians(hfov_deg) / 2.0)
    v = ((np.arange(rows) + 0.5) / rows * 2.0 - 1.0) * t * aspect
    p = math.radians(pitch_deg)
    down = (v + math.tan(p)) / (1.0 - v * math.tan(p))
    left = -((np.arange(cols) + 0.5) / cols * 2.0 - 1.0) * t
    return down, left


def lidar_scale(disp, down, left, points, camera_height, lidar_height, min_points=8,
                max_spread=0.25, max_range=4.0):
    """AI の値（disp、距離の逆数に比例）を m にする倍率 k（距離 = k / disp）。

    points は LiDAR の点の（カメラから見た方位 [rad]・左が正, 前方の距離 [m]）。LiDAR の光は
    カメラより lidar_height - camera_height 高いので、その点が写る行と列の値と比べる。
    点ごとの k の中央値を使う。点が min_points より少ないか、点ごとの k のばらつき
    （中央値からの差の中央値の割合）が max_spread より大きいときは k = None。
    返り値は (k, 使った点の数, ばらつき)。
    """
    rows, cols = disp.shape
    rise = lidar_height - camera_height
    ks = []
    for bearing, x in points:
        if not 0.15 <= x <= max_range:
            continue
        r = int(np.searchsorted(down, -rise / x))
        c = int(np.searchsorted(-left, -math.tan(bearing)))
        if not (1 <= r < rows - 1 and 1 <= c < cols - 1):
            continue
        ks.append(x * float(np.median(disp[r - 1:r + 2, c - 1:c + 2])))
    if len(ks) < min_points:
        return None, len(ks), None
    ks = np.asarray(ks)
    k = float(np.median(ks))
    spread = float(np.median(np.abs(ks / k - 1.0)))
    return (k if spread <= max_spread else None), len(ks), spread


def floor_ratio(depth, obst, down, camera_height, near=0.2, far=1.0):
    """床の行（near〜far m）の、m にした AI の距離 ÷ 床の三角形の距離の中央値。1 に近いほど合う。"""
    floor_dist = camera_height / np.where(down > 0.0, down, np.nan)
    rows = np.nonzero((floor_dist >= near) & (floor_dist <= far))[0]
    if not len(rows):
        return None
    ratios = depth[rows] / floor_dist[rows, None]
    ratios = ratios[~obst[rows]]
    return float(np.median(ratios)) if ratios.size else None


def obstacle_columns(disp, down, camera_height, nearer=1.15, min_height=0.02, fill=0.7,
                     min_down=0.02):
    """列ごとの、床から出っぱった物の一番下の行（なければ -1）と上の端の行と、物の画素。

    同じ行の中央値（ほとんど床）の nearer 倍より手前（disp が大きい）の画素を「物」とする。
    一番下の行 v の床の距離 D で、高さ min_height までの行（下向きの tan が v の
    (1 - min_height / カメラの高さ) 倍）の fill 以上が物なら、その列の物とする。
    物の一番下は、物の画素の奥行きの中央値に、床（行の中央値）が追いつく行とする。
    """
    rows, cols = disp.shape
    ok_rows = down > min_down
    ref = np.median(disp, axis=1)[:, None]
    obst = (disp > nearer * ref) & ok_rows[:, None]
    cs = np.vstack([np.zeros((1, cols)), np.cumsum(obst, axis=0)])
    top_need = np.searchsorted(down, down * (1.0 - min_height / camera_height))
    top_need = np.clip(top_need, 0, rows - 1)
    span = (np.arange(rows) - top_need + 1).astype(np.float64)[:, None]
    frac = (cs[1:] - cs[top_need]) / np.maximum(span, 1.0)
    cand = obst & (frac >= fill)
    has = cand.any(axis=0)
    bottom = np.where(has, rows - 1 - np.argmax(cand[::-1], axis=0), -1)
    top = bottom.copy()
    for c in np.nonzero(has)[0]:
        free = np.nonzero(~obst[:bottom[c], c])[0]
        top[c] = free[-1] + 1 if len(free) else 0
        # AI は物の下のはしを床にとけこませるので、物の奥行きと同じ値になる床の行まで下げる
        lo = max(top[c], bottom[c] - 3)
        col = disp[lo:bottom[c] + 1, c]
        level = np.median(col[obst[lo:bottom[c] + 1, c]])
        below = np.nonzero(ref[bottom[c]:, 0] >= level)[0]
        if len(below):
            bottom[c] += below[0]
    return bottom, top, obst


def group_objects(bottom, top, down, left, camera_height, max_range, min_cols=3,
                  min_width=0.02, gap_ratio=0.15, depth=None, obst=None):
    """となりの列で距離が近いものをまとめて、物のリスト（近い順）にする。

    depth（m にした AI の奥行き）と obst があれば、列の距離は物の画素の depth の中央値。
    なければ、物が床につく行の床の距離。どちらでも床の距離を floor_distance_m に入れる。
    """
    cols = len(bottom)
    dist = np.full(cols, np.nan)
    has = bottom >= 0
    floor = np.full(cols, np.nan)
    floor[has] = camera_height / down[bottom[has]]
    if depth is None:
        dist = floor.copy()
    else:
        for c in np.nonzero(has)[0]:
            lo = max(top[c], 0)
            col = depth[lo:bottom[c] + 1, c][obst[lo:bottom[c] + 1, c]]
            dist[c] = float(np.median(col)) if col.size else floor[c]
    dist[dist > max_range] = np.nan
    groups, cur = [], []
    for c in range(cols):
        d = dist[c]
        if np.isnan(d):
            if cur:
                groups.append(cur)
            cur = []
            continue
        if cur and abs(d - dist[cur[-1]]) > max(0.05, gap_ratio * d):
            groups.append(cur)
            cur = []
        cur.append(c)
    if cur:
        groups.append(cur)
    found = []
    for g in groups:
        g = np.asarray(g)
        d = dist[g]
        lateral = d * left[g]
        width = float(lateral.max() - lateral.min())
        if len(g) < min_cols or width < min_width:
            continue
        near = float(np.percentile(d, 10))
        heights = camera_height - d * np.maximum(down[np.maximum(top[g], 0)], 0.0)
        bearing = np.degrees(np.arctan(left[g]))
        found.append({
            'distance_m': round(near, 3),
            'floor_distance_m': round(float(np.nanpercentile(floor[g], 10)), 3),
            'lateral_m': round(float(np.median(lateral)), 3),
            'height_m': round(float(np.median(heights)), 3),
            'width_m': round(width, 3),
            'x_min': round(float(g.min()) / cols, 3),
            'x_max': round(float(g.max() + 1) / cols, 3),
            'bearing_min_deg': round(float(bearing.min()), 1),
            'bearing_max_deg': round(float(bearing.max()), 1),
            'depression_min_deg': round(math.degrees(math.atan(
                max(float(np.median(down[np.maximum(top[g], 0)])), 0.0))), 1),
            'depression_max_deg': round(math.degrees(math.atan(
                float(np.max(down[bottom[g]])))), 1),
            'points': int(len(g)),
        })
    found.sort(key=lambda o: o['distance_m'])
    return found


def in_corridor(objects, center_y, half_width):
    """通り道（左右 center_y ± half_width）にかかる物のうち、いちばん近い物。なければ None。"""
    for obj in objects:
        d = obj['distance_m']
        lo = d * math.tan(math.radians(obj['bearing_min_deg']))
        hi = d * math.tan(math.radians(obj['bearing_max_deg']))
        if lo <= center_y + half_width and hi >= center_y - half_width:
            return obj
    return None


class ObjectTracker:
    """何枚かの結果を合わせて、物の出たり消えたりと距離のゆれをおさえる。

    同じ物（横の範囲が重なり、距離が近い）を前の枚と結ぶ。window_s 秒の中で confirm 回
    見えたら出し、見えなくなっても hold_s 秒は前の値で出しつづける。距離は最近 history 回の
    中央値とし、今の値のほうが近ければ今の値を使う（近づいた物はすぐ近く出す）。
    """

    def __init__(self, confirm=2, window_s=0.5, hold_s=0.4, history=5):
        self.confirm = confirm
        self.window_s = window_s
        self.hold_s = hold_s
        self.history = history
        self._tracks = []

    def _match(self, track, obj):
        last = track['obj']
        if obj['x_min'] > last['x_max'] + 0.02 or obj['x_max'] < last['x_min'] - 0.02:
            return False
        d = obj['distance_m']
        return abs(d - track['dists'][-1]) <= max(0.08, 0.2 * d)

    def update(self, objects, now):
        free = list(self._tracks)
        for obj in objects:
            best = None
            for t in free:
                if self._match(t, obj) and (best is None or abs(
                        t['dists'][-1] - obj['distance_m']) < abs(
                        best['dists'][-1] - obj['distance_m'])):
                    best = t
            if best is None:
                best = {'dists': deque(maxlen=self.history), 'seen': deque()}
                self._tracks.append(best)
            else:
                free.remove(best)
            best['obj'] = obj
            best['dists'].append(obj['distance_m'])
            best['seen'].append(now)
            best['fresh'] = True
        for t in free:
            t['fresh'] = False
        out = []
        keep = []
        for t in self._tracks:
            if now - t['seen'][-1] > self.hold_s:
                continue
            keep.append(t)
            while t['seen'] and now - t['seen'][0] > self.window_s:
                t['seen'].popleft()
            if len(t['seen']) < self.confirm and not t.get('shown'):
                continue
            t['shown'] = True
            smooth = float(np.median(t['dists']))
            d = min(smooth, t['dists'][-1]) if t['fresh'] else smooth
            obj = dict(t['obj'])
            if d != obj['distance_m']:
                k = d / obj['distance_m']
                obj['lateral_m'] = round(obj['lateral_m'] * k, 3)
                obj['width_m'] = round(obj['width_m'] * k, 3)
                obj['distance_m'] = round(d, 3)
            obj['held'] = not t['fresh']
            out.append(obj)
        self._tracks = keep
        out.sort(key=lambda o: o['distance_m'])
        return out


def colorize(disp, obst, size):
    """奥行きの色の画像（近い = 赤、遠い = 青、床から出っぱった所 = 白で混ぜる）。"""
    lo, hi = np.percentile(disp, [2, 98])
    t = np.clip((disp - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    img = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[obst] = (img[obst] // 2 + 127).astype(np.uint8)
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
