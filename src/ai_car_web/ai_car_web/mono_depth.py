"""1 眼カメラの奥行きの AI（SC-Depth v3）の出力から、床から出っぱった低い物を見つける。

AI の奥行きは「何倍か」がわからず（縮尺なし）、遠くほど縮んで出る。そこで距離は AI から
読まない。同じ行（床なら同じ距離）の真ん中の値より手前に見える所を「物」とし、物が床に
ついている一番下の行を、カメラの高さと下向きの角度から床の距離に直す。
"""

import math

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
                  min_width=0.02, gap_ratio=0.15):
    """となりの列で距離が近いものをまとめて、物のリスト（近い順）にする。"""
    cols = len(bottom)
    dist = np.full(cols, np.nan)
    has = bottom >= 0
    dist[has] = camera_height / down[bottom[has]]
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


def colorize(disp, obst, size):
    """奥行きの色の画像（近い = 赤、遠い = 青、床から出っぱった所 = 白で混ぜる）。"""
    lo, hi = np.percentile(disp, [2, 98])
    t = np.clip((disp - lo) / max(hi - lo, 1e-6), 0.0, 1.0)
    img = cv2.applyColorMap((t * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
    img[obst] = (img[obst] // 2 + 127).astype(np.uint8)
    return cv2.resize(img, size, interpolation=cv2.INTER_NEAREST)
