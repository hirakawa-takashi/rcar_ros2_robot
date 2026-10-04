#!/usr/bin/env python3
"""2 眼カメラのキャリブレーション（市松模様の内側の角を使う）。

stereo_camera_node の「撮影」で calib_dir に pair_NNN_left.png / pair_NNN_right.png を
ためてから、`ros2 run ai_car_web stereo_calibrate` で左右のレンズのゆがみ・向きのずれ・
レンズの間隔を求め、calib_dir/stereo_calib.yaml に書く。
"""

import argparse
import glob
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml

DEFAULT_DIR = os.path.expanduser('~/AI-CAR_ws/calib/stereo')


def find_board(image, pattern):
    """市松模様の内側の角（pattern = (横, 縦)）を探す。見つからなければ None。"""
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
    return corners if ok else None


def list_pairs(calib_dir):
    """左右がそろっている (left, right) の組を番号順に返す。"""
    pairs = []
    for left in sorted(glob.glob(os.path.join(calib_dir, 'pair_*_left.png'))):
        right = left[:-len('_left.png')] + '_right.png'
        if os.path.exists(right):
            pairs.append((left, right))
    return pairs


def next_index(calib_dir):
    nums = [int(os.path.basename(p)[5:8]) for p, _ in list_pairs(calib_dir)
            if os.path.basename(p)[5:8].isdigit()]
    return max(nums, default=0) + 1


def board_points(pattern, square_m):
    cols, rows = pattern
    obj = np.zeros((cols * rows, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return obj


def _rectified_y_error(pts_l, pts_r, k1, d1, k2, d2, r1, r2, p1, p2):
    """平行化したあとの左右の同じ角の上下のずれ [px]（0 に近いほどよい）。"""
    errs = []
    for a, b in zip(pts_l, pts_r):
        ua = cv2.undistortPoints(a, k1, d1, R=r1, P=p1)
        ub = cv2.undistortPoints(b, k2, d2, R=r2, P=p2)
        errs.append(np.abs(ua[:, 0, 1] - ub[:, 0, 1]))
    return float(np.mean(np.concatenate(errs)))


def calibrate(pairs, pattern, square_m, rational=False):
    """左右の組からステレオのキャリブレーションを求めて dict で返す。"""
    obj = board_points(pattern, square_m)
    objs, pts_l, pts_r, used, skipped = [], [], [], [], []
    size = None
    for left_path, right_path in pairs:
        left = cv2.imread(left_path, cv2.IMREAD_GRAYSCALE)
        right = cv2.imread(right_path, cv2.IMREAD_GRAYSCALE)
        if left is None or right is None:
            skipped.append(os.path.basename(left_path))
            continue
        size = size or (left.shape[1], left.shape[0])
        cl, cr = find_board(left, pattern), find_board(right, pattern)
        if cl is None or cr is None:
            skipped.append(os.path.basename(left_path))
            continue
        objs.append(obj)
        pts_l.append(cl.astype(np.float32))
        pts_r.append(cr.astype(np.float32))
        used.append(os.path.basename(left_path))
    if len(objs) < 3:
        raise ValueError(f'市松模様が左右とも見つかった組が {len(objs)} 組しかない（3 組以上、目安 20 組）')

    flags = cv2.CALIB_RATIONAL_MODEL if rational else 0
    rms_l, k1, d1, _, _ = cv2.calibrateCamera(objs, pts_l, size, None, None, flags=flags)
    rms_r, k2, d2, _, _ = cv2.calibrateCamera(objs, pts_r, size, None, None, flags=flags)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, k1, d1, k2, d2, rot, trans, _, _ = cv2.stereoCalibrate(
        objs, pts_l, pts_r, k1, d1, k2, d2, size,
        criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC)
    r1, r2, p1, p2, q, _, _ = cv2.stereoRectify(
        k1, d1, k2, d2, size, rot, trans, flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
    fx = float(k1[0, 0])
    return {
        'stamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'image_size': [int(size[0]), int(size[1])],
        'board': {'cols': pattern[0], 'rows': pattern[1], 'square_m': float(square_m)},
        'pairs_used': len(used),
        'pairs_skipped': skipped,
        'rms_left_px': float(rms_l),
        'rms_right_px': float(rms_r),
        'rms_stereo_px': float(rms),
        'rectified_y_error_px': _rectified_y_error(pts_l, pts_r, k1, d1, k2, d2, r1, r2, p1, p2),
        'baseline_m': float(np.linalg.norm(trans)),
        'hfov_deg': math.degrees(2 * math.atan(size[0] / (2 * fx))),
        'vfov_deg': math.degrees(2 * math.atan(size[1] / (2 * float(k1[1, 1])))),
        'model': 'rational' if rational else 'standard',
        'K1': k1.tolist(), 'D1': d1.ravel().tolist(),
        'K2': k2.tolist(), 'D2': d2.ravel().tolist(),
        'R': rot.tolist(), 'T': trans.ravel().tolist(),
        'R1': r1.tolist(), 'R2': r2.tolist(),
        'P1': p1.tolist(), 'P2': p2.tolist(), 'Q': q.tolist(),
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description='2 眼カメラのキャリブレーション')
    ap.add_argument('--dir', default=DEFAULT_DIR, help='撮影した組のあるフォルダ')
    ap.add_argument('--cols', type=int, default=9, help='市松模様の内側の角の数（横）')
    ap.add_argument('--rows', type=int, default=6, help='市松模様の内側の角の数（縦）')
    ap.add_argument('--square-mm', type=float, default=24.0, help='1 マスの大きさ（実測）[mm]')
    ap.add_argument('--rational', action='store_true', help='広角のゆがみが大きいとき（係数 8 個）')
    ap.add_argument('--output', default='', help='出力先（既定: <dir>/stereo_calib.yaml）')
    args = ap.parse_args(argv)

    pairs = list_pairs(args.dir)
    try:
        result = calibrate(pairs, (args.cols, args.rows), args.square_mm / 1000.0, args.rational)
    except ValueError as e:
        print(e, file=sys.stderr)
        return 1
    out = args.output or os.path.join(args.dir, 'stereo_calib.yaml')
    with open(out, 'w', encoding='utf-8') as f:
        yaml.safe_dump(result, f, allow_unicode=True, sort_keys=False)
    print(f'使った組: {result["pairs_used"]} / {len(pairs)}')
    print(f'誤差 [px]: 左 {result["rms_left_px"]:.3f} / 右 {result["rms_right_px"]:.3f} / '
          f'ステレオ {result["rms_stereo_px"]:.3f} / 平行化後の上下のずれ '
          f'{result["rectified_y_error_px"]:.3f}')
    print(f'レンズの間隔: {result["baseline_m"] * 1000:.1f} mm、画角: 横 {result["hfov_deg"]:.1f}° / '
          f'縦 {result["vfov_deg"]:.1f}°')
    print(f'書いた: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
