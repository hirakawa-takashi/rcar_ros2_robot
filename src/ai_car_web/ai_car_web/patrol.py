"""パトロールの計算（ROS なし）: 地図の上の「見る所」、回る順番、地図になかった床の物。

grid は dashboard_node._map_cb の _map_grid（OccupancyGrid の data を bytes で持ったもの）。
"""
import math

import numpy as np

FREE_MAX = 19      # map_server の trinary と同じ: 0〜19 を空きとみなす
OCCUPIED_MIN = 65


def _cells(grid):
    return np.frombuffer(grid['data'], dtype=np.int8).reshape(grid['height'], grid['width'])


def _to_world(grid, col, row):
    res, yaw = grid['resolution'], grid.get('origin_yaw') or 0.0
    lx, ly = (col + 0.5) * res, (row + 0.5) * res
    c, s = math.cos(yaw), math.sin(yaw)
    return grid['origin_x'] + c * lx - s * ly, grid['origin_y'] + s * lx + c * ly


def _to_cell(grid, x, y):
    yaw = grid.get('origin_yaw') or 0.0
    dx, dy = x - grid['origin_x'], y - grid['origin_y']
    c, s = math.cos(yaw), math.sin(yaw)
    return int(math.floor((c * dx + s * dy) / grid['resolution'])), int(math.floor((-s * dx + c * dy) / grid['resolution']))


def _dilate(mask, radius):
    """mask を radius マス（円）ふくらませる。"""
    out = mask.copy()
    h, w = mask.shape
    for dr in range(-radius, radius + 1):
        for dc in range(-radius, radius + 1):
            if (dr or dc) and dr * dr + dc * dc <= radius * radius:
                src = mask[max(0, -dr):h - max(0, dr), max(0, -dc):w - max(0, dc)]
                out[max(0, dr):h - max(0, -dr), max(0, dc):w - max(0, -dc)] |= src
    return out


def reachable_cells(grid, start, clear_m):
    """壁・不明から clear_m 以上はなれた空きのうち、start（x, y）からつながっている所（bool の配列）。"""
    cells = _cells(grid)
    free = (cells >= 0) & (cells <= FREE_MAX)
    ok = free & ~_dilate(~free, max(1, int(math.ceil(clear_m / grid['resolution']))))
    if not ok.any():
        return ok
    col, row = _to_cell(grid, *start)
    rows, cols = np.nonzero(ok)
    i = int(np.argmin((rows - row) ** 2 + (cols - col) ** 2))
    if math.hypot(rows[i] - row, cols[i] - col) * grid['resolution'] > 0.6:
        return np.zeros_like(ok)
    seen = np.zeros_like(ok)
    seen[rows[i], cols[i]] = True
    while True:
        grow = seen.copy()
        grow[1:, :] |= seen[:-1, :]
        grow[:-1, :] |= seen[1:, :]
        grow[:, 1:] |= seen[:, :-1]
        grow[:, :-1] |= seen[:, 1:]
        grow &= ok
        if grow.sum() == seen.sum():
            return seen
        seen = grow


def coverage_points(grid, start, spacing_m=1.5, clear_m=0.35, min_cells=8):
    """地図を spacing_m の升に分け、升ごとに 1 つ、車が行ける所でいちばん升のまん中に近い所を「見る所」にする。"""
    reach = reachable_cells(grid, start, clear_m)
    step = max(1, int(round(spacing_m / grid['resolution'])))
    h, w = reach.shape
    out = []
    for r0 in range(0, h, step):
        for c0 in range(0, w, step):
            block = reach[r0:r0 + step, c0:c0 + step]
            if block.sum() < min_cells:
                continue
            rows, cols = np.nonzero(block)
            cr, cc = (block.shape[0] - 1) / 2, (block.shape[1] - 1) / 2
            i = int(np.argmin((rows - cr) ** 2 + (cols - cc) ** 2))
            x, y = _to_world(grid, c0 + cols[i], r0 + rows[i])
            out.append({'x': round(x, 2), 'y': round(y, 2)})
    return out


def _length(start, pts):
    total, (px, py) = 0.0, start
    for p in pts:
        total += math.hypot(p['x'] - px, p['y'] - py)
        px, py = p['x'], p['y']
    return total


def order_points(start, pts):
    """start から全部を 1 回ずつ回る順番。いちばん近い所から順に決めて、2-opt で交差をほどく（まっすぐの距離）。"""
    rest, order, (px, py) = list(pts), [], start
    while rest:
        i = min(range(len(rest)), key=lambda k: math.hypot(rest[k]['x'] - px, rest[k]['y'] - py))
        p = rest.pop(i)
        order.append(p)
        px, py = p['x'], p['y']
    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                cand = order[:i] + order[i:j + 1][::-1] + order[j + 1:]
                if _length(start, cand) < _length(start, order) - 1e-6:
                    order, improved = cand, True
    return order


def new_objects(grid, pose, points, lidar_offset=(0.0, 0.0), max_range_m=2.5, margin=2, cluster_m=0.2, min_points=3):
    """LiDAR の点（車の前が +x の [x, y]）のうち、地図では空きだった所（まわり margin マスも空き）に当たった点のかたまり。

    [{'x', 'y', 'n'}]。地図の外・不明・壁の近くの点は数えない。"""
    if not points or pose is None:
        return []
    cells = _cells(grid)
    h, w = cells.shape
    yaw = math.radians(pose['yaw_deg'])
    c, s = math.cos(yaw), math.sin(yaw)
    hits = []
    for px, py in points:
        if math.hypot(px, py) > max_range_m:
            continue
        lx, ly = px + lidar_offset[0], py + lidar_offset[1]
        x, y = pose['x'] + c * lx - s * ly, pose['y'] + s * lx + c * ly
        col, row = _to_cell(grid, x, y)
        if not (margin <= row < h - margin and margin <= col < w - margin):
            continue
        around = cells[row - margin:row + margin + 1, col - margin:col + margin + 1]
        if ((around >= 0) & (around <= FREE_MAX)).all():
            hits.append((x, y))
    clusters = []
    for x, y in hits:
        for cl in clusters:
            if math.hypot(cl['sx'] / cl['n'] - x, cl['sy'] / cl['n'] - y) <= cluster_m:
                cl['sx'] += x
                cl['sy'] += y
                cl['n'] += 1
                break
        else:
            clusters.append({'sx': x, 'sy': y, 'n': 1})
    return [{'x': round(cl['sx'] / cl['n'], 2), 'y': round(cl['sy'] / cl['n'], 2), 'n': cl['n']}
            for cl in clusters if cl['n'] >= min_points]
