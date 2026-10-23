#!/usr/bin/env python3
"""自律走行ノード（Level 1: LiDAR 反応型、Level 2: Nav2 で地図の行き先へ）。

行き先がないときは、地図や自己位置を使わず、/scan と /obstacle_status だけで
  前進 → 接近したら減速 → 停止距離内なら空いている側へ旋回 → 前進
を繰り返す（Level 1）。前の近さは、車の幅（path_half_width）の帯の中の点だけで決める
（ドアの柱のように帯の外の物では減速・旋回しない）。左右の両方に物が近いとき（ドア・廊下）は、
向きを変えずに横へずれて（linear.y）まん中を通る。減速に入ったら空いている側を 1 回だけ決め、
近づくほど強くその側へ曲がる（止まってから回る前に、向きを変えはじめる）。
動く指令を出しているのに stuck_sec 秒、LiDAR の距離も向き（/odom の yaw）も変わらなければ
「動けない」とみて、下がる → 横にずれる → 反対へ回る、で抜け出す（unstick_max 回まで）。
/nav_goal（JSON: {"action": "plan" | "go" | "cancel", "x", "y", "yaw_deg"}、map 座標）で行き先を受けると、
plan は Nav2 の compute_path_to_pose で道だけを考え（車は動かない）、go は navigate_to_pose で行き先へ向かう（Level 2）。
go のあいだは Nav2 の速度指令（/cmd_vel_nav）を中継し、着いたら・行けなかったら・やめたら止まって待つ。
AUTO を抜けると行き先をやめ、AUTO に入らないまま nav_auto_wait 秒たっても行き先をやめる。
段差から離れる動き（cliff_back / cliff_turn）は行き先より先にする。
速度指令は /cmd_vel_auto に常時（20Hz）publish し、
drive_mode_node が AUTO モードのときだけ /cmd_vel へ中継する。
AUTO 以外のモードでは状態を "standby" にしてゼロ速度を出し続ける
（drive_mode_node の生存確認に使われる）。

行動（/autonomy_status の behavior）:
  standby   AUTO 待機（ゼロ速度）
  forward   前進
  slow      減速前進（forward / slow は、左右の両方が近ければ横にずれてまん中へ）
  turn      その場旋回で空いている方向を探す
  backoff   前後左右とも近いときに少し後退
  cliff_back  前の落下防止センサーが段差 → /odom で cliff_backoff_m 下がる
  cliff_turn  段差から下がったあと、空いている側へ cliff_turn_deg 回って走行を続ける
  cliff_stuck 前後とも段差・/odom なしなどで動けない（ゼロ速度）
  stuck_back / stuck_side / stuck_turn  動けないので、下がる / 横にずれる / 反対へ回る
  stuck     抜け出しを unstick_max 回しても動けない（ゼロ速度。AUTO に入り直すまで）
  （段差の所は /odom と SLAM の地図の上で覚え、その所は前の障害物と同じにみる。曲がる先・横へずれる向きにも選ばない）
  tilted    車が AUTO に入ったときより tilt_stop_deg 以上かたむいた（落ちた・乗り上げた。ゼロ速度。AUTO に入り直すまで。その所を段差の印にする）
  blocked   LiDAR 未受信などで停止
  nav       Nav2 で地図の行き先へ向かう（/cmd_vel_nav を中継）
  nav_done  行き先に着いた（ゼロ速度。AUTO を抜けるか次の行き先まで）
  nav_failed  Nav2 が行き先へ行けなかった（ゼロ速度）
  nav_stop  行き先をやめた（ゼロ速度）
"""

import json
import math
import os
import random
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String

LABELS = {
    'standby': '待機', 'forward': '前進', 'slow': '減速', 'turn': '旋回',
    'backoff': '後退', 'blocked': '停止（センサー待ち）',
    'cliff_back': '段差から後退', 'cliff_turn': '段差から旋回',
    'cliff_stuck': '停止（段差で動けない）',
    'stuck_back': '抜け出し（後退）', 'stuck_side': '抜け出し（横）', 'stuck_turn': '抜け出し（旋回）',
    'stuck': '停止（動けない）',
    'tilted': '停止（かたむいた）',
    'nav': '行き先へ', 'nav_done': '行き先に着いた', 'nav_failed': '停止（行き先へ行けない）',
    'nav_stop': '停止（行き先をやめた）',
}
NAV_LABELS = {
    'idle': 'なし', 'unavailable': 'Nav2 が動いていない', 'planning': '道を考え中',
    'planned': '道あり（まだ動かない）', 'no_path': '道が見つからない', 'sending': '送り中',
    'active': '向かっている', 'succeeded': '着いた', 'aborted': '行けなかった', 'canceled': 'やめた',
}
UNSTICK_PHASES = ('back', 'side', 'turn')
UNSTICK_NAMES = {'back': '下がる', 'side': '横にずれる', 'turn': '反対へ回る'}
NAV_END = {'succeeded': 'nav_done', 'aborted': 'nav_failed', 'canceled': 'nav_stop'}
MAX_PATH_POINTS = 120


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _reachable(ok, row, col, search):
    """ok の True のマスを 4 近傍でたどって、(row, col) のいちばん近くの ok のマス（search マス以内）から行ける所。"""
    h, w = ok.shape
    r0, r1, c0, c1 = max(row - search, 0), min(row + search + 1, h), max(col - search, 0), min(col + search + 1, w)
    if r0 >= r1 or c0 >= c1:
        return None
    rr, cc = np.nonzero(ok[r0:r1, c0:c1])
    if len(rr) == 0:
        return None
    k = int(np.argmin((rr + r0 - row) ** 2 + (cc + c0 - col) ** 2))
    reach = np.zeros_like(ok)
    reach[rr[k] + r0, cc[k] + c0] = True
    while True:
        grow = reach.copy()
        grow[1:, :] |= reach[:-1, :]
        grow[:-1, :] |= reach[1:, :]
        grow[:, 1:] |= reach[:, :-1]
        grow[:, :-1] |= reach[:, 1:]
        grow &= ok
        if grow.sum() == reach.sum():
            return reach
        reach = grow


def _clusters(mask, min_cells):
    """mask の True のつながったかたまり（8 近傍）のうち min_cells 以上のものの、まん中にいちばん近いマス (行, 列)。"""
    cells = set(zip(*np.nonzero(mask)))
    out = []
    while cells:
        seed = cells.pop()
        group, stack = [seed], [seed]
        while stack:
            cy, cx = stack.pop()
            for ny in (cy - 1, cy, cy + 1):
                for nx in (cx - 1, cx, cx + 1):
                    if (ny, nx) in cells:
                        cells.remove((ny, nx))
                        group.append((ny, nx))
                        stack.append((ny, nx))
        if len(group) < min_cells:
            continue
        gy = sum(c[0] for c in group) / len(group)
        gx = sum(c[1] for c in group) / len(group)
        out.append(min(group, key=lambda c: (c[0] - gy) ** 2 + (c[1] - gx) ** 2))
    return out


def _path_points(path: Path):
    """道を MAX_PATH_POINTS 点までに間引いた [[x, y], ...] と長さ [m]。"""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    step = max(1, math.ceil(len(pts) / MAX_PATH_POINTS))
    keep = pts[::step]
    if pts and keep[-1] != pts[-1]:
        keep.append(pts[-1])
    return [[round(x, 2), round(y, 2)] for x, y in keep], length


def _slam_started():
    """slam_toolbox のプロセスが動きはじめた時刻（time.time()）。なければ None。"""
    try:
        with open('/proc/uptime') as f:
            boot = time.time() - float(f.read().split()[0])
        tick = os.sysconf('SC_CLK_TCK')
        for pid in os.listdir('/proc'):
            if not pid.isdigit():
                continue
            try:
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    if b'slam_toolbox' not in f.read().split(b'\0')[0]:
                        continue
                with open(f'/proc/{pid}/stat') as f:
                    return boot + int(f.read().rsplit(')', 1)[1].split()[19]) / tick
            except (OSError, ValueError, IndexError):
                continue
    except (OSError, ValueError):
        pass
    return None


class AutonomyNode(Node):
    """LiDAR 反応型の自律走行。"""

    def __init__(self):
        super().__init__('autonomy_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('drive_mode_topic', '/drive_mode')
        self.declare_parameter('cmd_topic', '/cmd_vel_auto')
        self.declare_parameter('status_topic', '/autonomy_status')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('scan_max_age', 1.0)
        # 巡航速度 [m/s]・減速時の速度・旋回速度 [rad/s]
        self.declare_parameter('cruise_speed', 0.15)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('turn_speed', 0.6)
        self.declare_parameter('backoff_speed', 0.08)
        # 前方 ±front_angle/2 の距離で判定
        self.declare_parameter('front_angle_deg', 60.0)
        self.declare_parameter('stop_distance', 0.35)
        self.declare_parameter('slow_distance', 0.8)
        # 旋回後に前進を再開する前方クリア距離
        self.declare_parameter('clear_distance', 0.9)
        # 左右・後方もこの距離以内なら後退
        self.declare_parameter('backoff_distance', 0.25)
        # 旋回の最短継続時間（振動防止）
        self.declare_parameter('min_turn_sec', 0.6)
        # 左右の両方がこれより空いていれば、曲がる向きをランダムに選ぶ [m]（同じ所をくるくる回らないため）
        self.declare_parameter('random_side_distance', 0.4)
        # 旋回を 0〜この秒数だけランダムに長くする（向きを毎回少し変える）
        self.declare_parameter('turn_random_sec', 0.8)
        # 曲がる先の選び方: まわり 360° を explore_step_deg ごとに、車の幅で explore_min_free m 以上まっすぐ進める向きを候補にし、
        # 「遠くまで空いている × まだ行っていない（/odom の explore_cell m マスに入った回数が少ない）」ほど選ばれやすいランダムで選ぶ
        # 細い出口（車の幅＋十数 cm）は 10〜15° の幅でしか見えないので、5° ごとに調べる。前 explore_min_turn_deg 以内は候補にしない
        self.declare_parameter('explore_step_deg', 5.0)
        self.declare_parameter('explore_min_turn_deg', 15.0)
        self.declare_parameter('explore_min_free', 1.0)
        self.declare_parameter('explore_cell', 0.5)
        self.declare_parameter('explore_reach_deg', 5.0)
        self.declare_parameter('explore_turn_timeout', 8.0)
        # 細い通路の入口: まっすぐ explore_min_free 進めない向きでも、その向きで入口の手前（前の物の stop_distance 手前、
        # explore_via_min_m 以上）まで進み、そこから ±explore_via_deg に車の幅で explore_min_free 以上進めれば候補にする。
        # 選んだら、その向きまで回ったあと explore_via_sec 秒は向きを選びなおさずに、入口の手前までまっすぐ進む
        self.declare_parameter('explore_via', True)
        self.declare_parameter('explore_via_deg', 60.0)
        self.declare_parameter('explore_via_min_m', 0.3)
        self.declare_parameter('explore_via_sec', 10.0)
        # 減速から前進にもどるのは slow_distance + slow_hysteresis_m より空いてから。
        # slow_keep_sec 以内にまた減速したら、前と同じ向きに曲がる（向きを選びなおさない）
        self.declare_parameter('slow_hysteresis_m', 0.15)
        self.declare_parameter('slow_keep_sec', 3.0)
        # その場で turn_max_deg 回っても出口がなければ止まる（ぐるぐる回りつづけない）
        self.declare_parameter('turn_max_deg', 400.0)
        # 前の近さを見る帯の半分の幅 [m]（車の幅 0.28 m の半分 + 6 cm）
        self.declare_parameter('path_half_width', 0.20)
        # 左右の両方にこれより近い物があれば、まん中へ横にずれる [m]
        self.declare_parameter('center_range', 0.5)
        self.declare_parameter('center_gain', 0.5)
        self.declare_parameter('center_speed', 0.04)
        self.declare_parameter('center_deadband', 0.03)
        # 減速の帯で、近づくほど空いている側へ曲がる強さ（turn_speed に対する割合。止まる距離で最大）
        self.declare_parameter('pre_turn_gain', 0.5)
        # 動く指令なのに stuck_sec 秒、距離が stuck_move_m・向きが stuck_turn_deg も変わらなければ「動けない」
        self.declare_parameter('stuck_sec', 3.0)
        # Nav2 で向かっているときは、車輪で数えた位置がこの秒数 stuck_move_m も動かなければ「動けない」
        self.declare_parameter('nav_stuck_sec', 5.0)
        self.declare_parameter('stuck_move_m', 0.04)
        self.declare_parameter('stuck_turn_deg', 10.0)
        self.declare_parameter('unstick_back_sec', 1.5)
        self.declare_parameter('unstick_side_sec', 1.5)
        self.declare_parameter('unstick_turn_deg', 60.0)
        self.declare_parameter('unstick_turn_sec', 4.0)
        # この回数抜け出しても（unstick_window 秒のうちに）また動けなければ止まる
        self.declare_parameter('unstick_max', 3)
        self.declare_parameter('unstick_window', 30.0)
        # AUTO に入ったときより roll / pitch が tilt_stop_deg 以上、tilt_hold_sec つづけてかたむいたら止まる（落ちた・乗り上げた）
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('tilt_stop_deg', 15.0)
        self.declare_parameter('tilt_hold_sec', 0.5)
        # 前の段差センサーが反応した所を覚えて、そちらへ向かう向きは曲がる先に選ばない（この幅 m の帯に入る向き）
        self.declare_parameter('cliff_avoid_m', 0.4)
        # 段差の所は SLAM の地図（map）の上でも覚え、cliff_file に保存する（同じ起動のあいだ = 同じ地図のあいだ使う）
        self.declare_parameter('slam_pose_topic', '/pose')
        self.declare_parameter('cliff_file', '')
        self.declare_parameter('cliff_clear_topic', '/cliff_marks_clear')
        # ダッシュボードの地図で手で置く・消す段差の印（JSON {action: add / remove, x, y}、map）
        self.declare_parameter('cliff_edit_topic', '/cliff_marks_edit')
        # 段差（/cliff_status）を見たら、/odom で下がって回り、走行を続ける
        self.declare_parameter('cliff_topic', '/cliff_status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cliff_timeout', 1.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('cliff_backoff_m', 0.10)
        self.declare_parameter('cliff_turn_deg', 120.0)
        self.declare_parameter('cliff_escape_timeout', 20.0)
        # Level 2: Nav2 で地図の行き先へ。nav_goal_topic を空にすると無効
        self.declare_parameter('nav_goal_topic', '/nav_goal')
        self.declare_parameter('nav_cmd_topic', '/cmd_vel_nav')
        self.declare_parameter('nav_plan_topic', '/plan')
        self.declare_parameter('nav_frame', 'map')
        self.declare_parameter('nav_cmd_timeout', 0.5)
        self.declare_parameter('nav_max_angular', 0.5)
        self.declare_parameter('nav_auto_wait', 5.0)
        # まだ行っていない所のはし（SLAM の地図で、空きと不明のさかい）を行き先にして Nav2 で行く。新しいマスに
        # frontier_stale_sec 秒入らなかったら、いちばん近い「はし」（frontier_min_cells 以上のかたまり、壁から
        # frontier_clear_m 以上、frontier_min_dist m 以上先、段差の印から cliff_avoid_m + 0.2 m 以上）へ向かう。
        # 着いた・行けなかった・frontier_timeout_sec 秒たった・段差の印に近づいたら、ふつうの走行にもどる。
        # 行けなかった所の 0.6 m 以内は frontier_retry_sec 秒えらばない。frontier_map_topic を空にすると無効
        self.declare_parameter('frontier_map_topic', '/map')
        self.declare_parameter('frontier_stale_sec', 30.0)
        self.declare_parameter('frontier_min_cells', 6)
        self.declare_parameter('frontier_clear_m', 0.25)
        self.declare_parameter('frontier_min_dist', 0.8)
        self.declare_parameter('frontier_timeout_sec', 90.0)
        self.declare_parameter('frontier_retry_sec', 1800.0)
        # LiDAR で見えていても車が行っていない所（通ったマスから frontier_visited_m より遠い空き、
        # frontier_min_area_cells 以上のかたまり）も行き先にする。見えている所は「はし」にならないため
        self.declare_parameter('frontier_visited_m', 0.75)
        self.declare_parameter('frontier_min_area_cells', 40)

        g = self.get_parameter
        self.offset = math.radians(float(g('scan_angle_offset_deg').value))
        self.scan_max_age = float(g('scan_max_age').value)
        self.cruise = float(g('cruise_speed').value)
        self.slow = float(g('slow_speed').value)
        self.turn = float(g('turn_speed').value)
        self.backoff_speed = float(g('backoff_speed').value)
        self.front_half = math.radians(float(g('front_angle_deg').value)) / 2.0
        self.stop_d = float(g('stop_distance').value)
        self.slow_d = float(g('slow_distance').value)
        self.clear_d = float(g('clear_distance').value)
        self.backoff_d = float(g('backoff_distance').value)
        self.min_turn = float(g('min_turn_sec').value)
        self.random_side = float(g('random_side_distance').value)
        self.turn_random = float(g('turn_random_sec').value)
        self._turn_extra = 0.0
        self.explore_step = math.radians(float(g('explore_step_deg').value))
        self.explore_min_turn = math.radians(float(g('explore_min_turn_deg').value))
        self.explore_min_free = float(g('explore_min_free').value)
        self.explore_cell = float(g('explore_cell').value)
        self.explore_reach = math.radians(float(g('explore_reach_deg').value))
        self.explore_turn_timeout = float(g('explore_turn_timeout').value)
        self.explore_via = bool(g('explore_via').value)
        via_deg = float(g('explore_via_deg').value)
        self._via_dth = np.radians(np.arange(-via_deg, via_deg + 1e-6, math.degrees(self.explore_step)))
        self.explore_via_min = float(g('explore_via_min_m').value)
        self.explore_via_sec = float(g('explore_via_sec').value)
        self._via_pending = False
        self._via_until = 0.0
        self._via_pick = None
        self._scan_xy = np.zeros((0, 2))
        self.slow_hyst = float(g('slow_hysteresis_m').value)
        self.slow_keep = float(g('slow_keep_sec').value)
        self.turn_max = math.radians(float(g('turn_max_deg').value))
        self._slow_left = -float('inf')
        self._turn_acc = 0.0
        self._turn_yaw = None
        self._free = []          # [(向き rad, 車の幅で進める距離 m)]
        self._visits = {}        # /odom のマス → 入った回数
        self._last_cell = None
        self._target_yaw = None  # 曲がる先の向き（/odom の yaw）
        self.path_half = float(g('path_half_width').value)
        self.center_range = float(g('center_range').value)
        self.center_gain = float(g('center_gain').value)
        self.center_speed = float(g('center_speed').value)
        self.center_deadband = float(g('center_deadband').value)
        self.pre_turn_gain = float(g('pre_turn_gain').value)
        self.stuck_sec = float(g('stuck_sec').value)
        self.nav_stuck_sec = float(g('nav_stuck_sec').value)
        self.stuck_move = float(g('stuck_move_m').value)
        self.stuck_turn = math.radians(float(g('stuck_turn_deg').value))
        self.unstick_back_sec = float(g('unstick_back_sec').value)
        self.unstick_side_sec = float(g('unstick_side_sec').value)
        self.unstick_turn = math.radians(float(g('unstick_turn_deg').value))
        self.unstick_turn_sec = float(g('unstick_turn_sec').value)
        self.unstick_max = int(g('unstick_max').value)
        self.unstick_window = float(g('unstick_window').value)
        self.tilt_stop = math.radians(float(g('tilt_stop_deg').value))
        self.tilt_hold = float(g('tilt_hold_sec').value)
        self.cliff_avoid = float(g('cliff_avoid_m').value)
        self._tilt = None        # (roll, pitch) rad
        self._tilt_stamp = 0.0
        self._tilt_base = None   # AUTO に入ったときの (roll, pitch)
        self._tilt_since = None
        self._cliff_pts = []     # 段差があった所（/odom。地図の位置がまだ分からないとき）
        self._cliff_marks = []   # 段差があった所（map）
        self._map_odom = None    # map から見た odom (x, y, yaw)
        self.cliff_file = g('cliff_file').value
        self._load_marks()
        self.cliff_timeout = float(g('cliff_timeout').value)
        self.odom_timeout = float(g('odom_timeout').value)
        self.cliff_backoff_m = float(g('cliff_backoff_m').value)
        self.cliff_turn = math.radians(float(g('cliff_turn_deg').value))
        self.cliff_escape_timeout = float(g('cliff_escape_timeout').value)
        self.nav_frame = g('nav_frame').value
        self.nav_cmd_timeout = float(g('nav_cmd_timeout').value)
        self.nav_max_angular = float(g('nav_max_angular').value)
        self.nav_auto_wait = float(g('nav_auto_wait').value)
        self.frontier_stale = float(g('frontier_stale_sec').value)
        self.frontier_min_cells = int(g('frontier_min_cells').value)
        self.frontier_clear = float(g('frontier_clear_m').value)
        self.frontier_min_dist = float(g('frontier_min_dist').value)
        self.frontier_timeout = float(g('frontier_timeout_sec').value)
        self.frontier_retry = float(g('frontier_retry_sec').value)
        self.frontier_visited = float(g('frontier_visited_m').value)
        self.frontier_min_area = int(g('frontier_min_area_cells').value)
        self._grid = None          # (占有の配列 [行=y, 列=x], 解像度, 原点 x, 原点 y)
        self._frontier = None      # 向かっている「はし」 {'x', 'y', 'since'}
        self._frontier_bad = []    # [(x, y, いつ)] 行けなかった「はし」
        self._frontier_tried = 0.0
        self._frontier_count = 0
        self._new_cell_at = time.monotonic()

        self._lock = threading.Lock()
        self._mode = None
        self._sectors = None   # {'front','left','right','back'} 最短距離
        self._scan_stamp = 0.0
        self._obstacle = None
        self._obstacle_stamp = 0.0
        self._behavior = 'standby'
        self._behavior_since = time.monotonic()
        self._turn_dir = 1.0
        self._pre_dir = None
        self._last_cmd = Twist()
        self._cliff = None
        self._cliff_stamp = 0.0
        self._pose = None      # (x, y, yaw)
        self._pose_stamp = 0.0
        self._escape = None    # 段差から離れる途中の状態
        self._progress = None  # 動けているかを見るための、少し前の距離と向き
        self._unstick = None   # 動けないときの抜け出しの途中の状態
        self._unstick_times = []
        self._prev_mode = None
        self._nav = self._nav_state('idle')
        self._nav_seq = 0      # 古い行き先の返事を捨てるための番号
        self._nav_handle = None
        self._nav_hold = None  # 着いた・行けなかった・やめたあと止まって待つときの behavior
        self._nav_cmd = Twist()
        self._nav_cmd_stamp = 0.0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=5)
        self.cmd_pub = self.create_publisher(Twist, g('cmd_topic').value, 10)
        self.status_pub = self.create_publisher(String, g('status_topic').value, 10)
        self.create_subscription(LaserScan, g('scan_topic').value, self._scan_cb, sensor_qos)
        self.create_subscription(String, g('obstacle_topic').value, self._obstacle_cb, 10)
        if g('cliff_topic').value:
            self.create_subscription(String, g('cliff_topic').value, self._cliff_cb, 10)
        if g('odom_topic').value:
            self.create_subscription(Odometry, g('odom_topic').value, self._odom_cb, 10)
        if g('imu_topic').value:
            self.create_subscription(Imu, g('imu_topic').value, self._imu_cb, sensor_qos)
        if g('slam_pose_topic').value:
            self.create_subscription(PoseWithCovarianceStamped, g('slam_pose_topic').value,
                                     self._slam_pose_cb, 10)
        if g('cliff_clear_topic').value:
            self.create_subscription(String, g('cliff_clear_topic').value, self._cliff_clear_cb, 10)
        if g('cliff_edit_topic').value:
            self.create_subscription(String, g('cliff_edit_topic').value, self._cliff_edit_cb, 10)
        self.create_subscription(
            String, g('drive_mode_topic').value, self._mode_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.nav_client = self.plan_client = None
        if g('nav_goal_topic').value:
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.plan_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
            self.create_subscription(String, g('nav_goal_topic').value, self._nav_goal_cb, 10)
            self.create_subscription(Twist, g('nav_cmd_topic').value, self._nav_cmd_cb, 10)
            self.create_subscription(Path, g('nav_plan_topic').value, self._nav_plan_cb, 10)
            if g('frontier_map_topic').value:
                self.create_subscription(
                    OccupancyGrid, g('frontier_map_topic').value, self._grid_cb,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE))
        rate = float(g('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        self.create_timer(0.2, self._status_cb)

    # --- 入力 ---
    def _scan_cb(self, msg: LaserScan):
        sectors = {'front': [], 'left': [], 'right': [], 'back': []}
        path = []
        side_l, side_r = [], []
        pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + self.offset
            a = math.atan2(math.sin(a), math.cos(a))
            x, y = r * math.cos(a), r * math.sin(a)
            pts.append((x, y))
            if x > 0.0 and abs(y) <= self.path_half:
                path.append(x)
            elif -0.2 <= x <= 0.4 and abs(y) > 0.1:
                (side_l if y > 0.0 else side_r).append(abs(y))
            if abs(a) <= self.front_half:
                sectors['front'].append((a, r))
            elif abs(a) >= math.pi - self.front_half:
                sectors['back'].append((a, r))
            elif a > 0:
                sectors['left'].append((a, r))
            else:
                sectors['right'].append((a, r))
        summary = {}
        for k, v in sectors.items():
            summary[k] = min(r for _, r in v) if v else None
        # 前方セクター内で左半分・右半分のどちらが空いているか
        fl = [r for a, r in sectors['front'] if a > 0]
        fr = [r for a, r in sectors['front'] if a <= 0]
        summary['front_left'] = min(fl) if fl else None
        summary['front_right'] = min(fr) if fr else None
        # 車の幅の帯の中で、いちばん近い物までの前の距離と、車の横（前 0.4 m〜後 0.2 m）の物までの左右の距離
        summary['path'] = min(path) if path else None
        summary['side_left'] = min(side_l) if side_l else None
        summary['side_right'] = min(side_r) if side_r else None
        n = max(1, int(round(2 * math.pi / self.explore_step)))
        ths = -math.pi + np.arange(n) * (2 * math.pi / n)
        free_d = np.full(n, 4.0)
        if pts:
            xy = np.asarray(pts)
            along = xy[:, :1] * np.cos(ths) + xy[:, 1:] * np.sin(ths)
            perp = xy[:, 1:] * np.cos(ths) - xy[:, :1] * np.sin(ths)
            hit = np.where((along > 0.0) & (np.abs(perp) <= self.path_half), along, 4.0)
            free_d = np.minimum(hit.min(axis=0), 4.0)
        free = list(zip(ths.tolist(), free_d.tolist()))
        with self._lock:
            self._sectors = summary
            self._free = free
            self._scan_xy = np.asarray(pts) if pts else np.zeros((0, 2))
            self._scan_stamp = time.monotonic()

    def _obstacle_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            with self._lock:
                self._obstacle = payload
                self._obstacle_stamp = time.monotonic()

    def _cliff_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            with self._lock:
                self._cliff = payload
                self._cliff_stamp = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        with self._lock:
            self._pose = (p.position.x, p.position.y, yaw)
            self._pose_stamp = time.monotonic()

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        with self._lock:
            self._tilt = (roll, pitch)
            self._tilt_stamp = time.monotonic()

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        ys = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
        with self._lock:
            odom = self._pose
            if odom is None or time.monotonic() - self._pose_stamp > 0.5:
                return
            yaw = _wrap(ys - odom[2])
            c, s = math.cos(yaw), math.sin(yaw)
            self._map_odom = (p.position.x - (c * odom[0] - s * odom[1]),
                              p.position.y - (s * odom[0] + c * odom[1]), yaw)

    def _cliff_clear_cb(self, _msg: String):
        self._cliff_pts = []
        self._cliff_marks = []
        self._save_marks()
        self.get_logger().info('地図を消したので、段差の印も消した')

    def _cliff_edit_cb(self, msg: String):
        try:
            req = json.loads(msg.data)
            action, x, y = req['action'], float(req['x']), float(req['y'])
        except (TypeError, ValueError, KeyError, json.JSONDecodeError):
            return
        if action == 'add':
            if any(math.hypot(x - mx, y - my) < 0.05 for mx, my in self._cliff_marks):
                return
            self._cliff_marks = (self._cliff_marks + [(x, y)])[-50:]
        elif action == 'remove':
            self._cliff_marks = [(mx, my) for mx, my in self._cliff_marks if math.hypot(x - mx, y - my) >= 0.1]
        else:
            return
        self._save_marks()
        self.get_logger().info(f'段差の印を手で{"置いた" if action == "add" else "消した"}（地図 {x:.2f}, {y:.2f}）')

    def _load_marks(self):
        """今の slam_toolbox が動きはじめてから保存した段差の印だけ読む（地図は slam_toolbox の起動ごとに作り直すため）。"""
        if not self.cliff_file:
            return
        try:
            started = _slam_started()
            if started is None or os.path.getmtime(self.cliff_file) < started:
                return
            with open(self.cliff_file) as f:
                self._cliff_marks = [tuple(m[:2]) for m in json.load(f).get('marks', [])][-50:]
        except (OSError, ValueError, TypeError, AttributeError):
            self._cliff_marks = []

    def _save_marks(self):
        if not self.cliff_file:
            return
        try:
            os.makedirs(os.path.dirname(self.cliff_file) or '.', exist_ok=True)
            with open(self.cliff_file, 'w') as f:
                json.dump({'frame': 'map', 'marks': [[round(x, 3), round(y, 3)] for x, y in self._cliff_marks]}, f)
        except OSError as exc:
            self.get_logger().warn(f'段差の印を保存できない: {exc}')

    def _add_cliff(self, pose, ahead=0.2):
        """段差の所（車の ahead m 前）を覚える。地図の位置が分かれば map で。"""
        ox, oy = pose[0] + ahead * math.cos(pose[2]), pose[1] + ahead * math.sin(pose[2])
        mo = self._map_odom
        if mo is None:
            self._cliff_pts = (self._cliff_pts + [(ox, oy)])[-50:]
            return
        c, s = math.cos(mo[2]), math.sin(mo[2])
        mx, my = mo[0] + c * ox - s * oy, mo[1] + s * ox + c * oy
        if any(math.hypot(mx - x, my - y) < 0.15 for x, y in self._cliff_marks):
            return
        self._cliff_marks = (self._cliff_marks + [(mx, my)])[-50:]
        self._save_marks()

    def _known_cliffs(self):
        """覚えている段差の所（/odom）。"""
        pts = list(self._cliff_pts)
        mo = self._map_odom
        if mo is not None and self._cliff_marks:
            c, s = math.cos(mo[2]), math.sin(mo[2])
            for mx, my in self._cliff_marks:
                dx, dy = mx - mo[0], my - mo[1]
                pts.append((c * dx + s * dy, -s * dx + c * dy))
        return pts

    def _cliff_ahead(self, pose):
        """まっすぐ前の車の幅の帯にある、覚えている段差までの距離（なければ inf）。"""
        if pose is None:
            return float('inf')
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        best = float('inf')
        for cx, cy in self._known_cliffs():
            dx, dy = cx - pose[0], cy - pose[1]
            along = dx * c + dy * sn
            # 車の前のはし（0.2 m）より手前の印は、もう車の下なので見ない（前の段差センサーにまかせる）
            if along > 0.2 and abs(dy * c - dx * sn) <= self.path_half + 0.1:
                best = min(best, along)
        return best

    def _spun_out(self, pose) -> bool:
        """その場の旋回で回った角度をたして、turn_max をこえたら True。"""
        if pose is None:
            return False
        if self._turn_yaw is not None:
            self._turn_acc += abs(_wrap(pose[2] - self._turn_yaw))
        self._turn_yaw = pose[2]
        return self._turn_acc > self.turn_max

    def _tilted(self, now, tilt) -> bool:
        """AUTO に入ったときより tilt_stop 以上のかたむきが tilt_hold 秒つづいたら True。"""
        if tilt is None:
            return False
        if self._tilt_base is None:
            self._tilt_base = tilt
        d = max(abs(_wrap(tilt[0] - self._tilt_base[0])), abs(tilt[1] - self._tilt_base[1]))
        if d < self.tilt_stop:
            self._tilt_since = None
            return False
        if self._tilt_since is None:
            self._tilt_since = now
        return now - self._tilt_since >= self.tilt_hold

    def _mode_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            with self._lock:
                self._mode = payload.get('mode')

    # --- Level 2: Nav2 ---
    def _nav_state(self, state, goal=None, reason='', path=None, length=None):
        return {'state': state, 'goal': goal, 'reason': reason, 'path': path or [],
                'path_m': round(length, 2) if length is not None else None,
                'remaining_m': None, 'recoveries': 0, 'since': time.monotonic()}

    def _nav_cmd_cb(self, msg: Twist):
        with self._lock:
            self._nav_cmd = msg
            self._nav_cmd_stamp = time.monotonic()

    def _nav_plan_cb(self, msg: Path):
        if self._nav['state'] != 'active' or not msg.poses:
            return
        pts, length = _path_points(msg)
        self._nav['path'] = pts
        self._nav['path_m'] = round(length, 2)

    def _nav_goal_cb(self, msg: String):
        try:
            req = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(req, dict):
            return
        action = req.get('action')
        if action == 'cancel':
            self._nav_cancel('やめる指示')
            return
        if action not in ('plan', 'go'):
            return
        try:
            x, y = float(req['x']), float(req['y'])
            yaw = math.radians(float(req.get('yaw_deg') or 0.0))
        except (KeyError, TypeError, ValueError):
            return
        goal = {'x': round(x, 2), 'y': round(y, 2), 'yaw_deg': round(math.degrees(yaw), 1)}
        client = self.nav_client if action == 'go' else self.plan_client
        self._nav_cancel(None)
        self._nav_seq += 1
        seq = self._nav_seq
        if not client.server_is_ready():
            self._nav = self._nav_state('unavailable', goal, 'Nav2 が active ではありません')
            return
        pose = PoseStamped()
        pose.header.frame_id = self.nav_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        if action == 'plan':
            self._nav = self._nav_state('planning', goal)
            req_msg = ComputePathToPose.Goal()
            req_msg.goal = pose
            req_msg.use_start = False
            fut = client.send_goal_async(req_msg)
            fut.add_done_callback(lambda f: self._plan_accepted(f, seq, goal))
        else:
            self._nav = self._nav_state('sending', goal, path=self._nav.get('path'))
            req_msg = NavigateToPose.Goal()
            req_msg.pose = pose
            fut = client.send_goal_async(req_msg, feedback_callback=self._nav_feedback)
            fut.add_done_callback(lambda f: self._nav_accepted(f, seq, goal))
        self.get_logger().info(f'行き先（{action}）: x {x:.2f} y {y:.2f}')

    def _plan_accepted(self, fut, seq, goal):
        handle = fut.result()
        if seq != self._nav_seq:
            return
        if not handle.accepted:
            self._nav = self._nav_state('no_path', goal, 'Nav2 が受け付けませんでした')
            return
        handle.get_result_async().add_done_callback(lambda f: self._plan_done(f, seq, goal))

    def _plan_done(self, fut, seq, goal):
        if seq != self._nav_seq:
            return
        res = fut.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED or not res.result.path.poses:
            self._nav = self._nav_state('no_path', goal, f'道が見つかりません（{res.result.error_code}）')
            return
        pts, length = _path_points(res.result.path)
        self._nav = self._nav_state('planned', goal, path=pts, length=length)

    def _nav_accepted(self, fut, seq, goal):
        handle = fut.result()
        if seq != self._nav_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self._nav = self._nav_state('aborted', goal, 'Nav2 が受け付けませんでした')
            return
        self._nav_handle = handle
        self._nav['state'] = 'active'
        self._nav['since'] = time.monotonic()
        handle.get_result_async().add_done_callback(lambda f: self._nav_done(f, seq, goal))

    def _nav_feedback(self, fb):
        if self._nav['state'] == 'active':
            self._nav['remaining_m'] = round(fb.feedback.distance_remaining, 2)
            self._nav['recoveries'] = fb.feedback.number_of_recoveries

    def _nav_done(self, fut, seq, goal):
        if seq != self._nav_seq:
            return
        res = fut.result()
        self._nav_handle = None
        state = {GoalStatus.STATUS_SUCCEEDED: 'succeeded',
                 GoalStatus.STATUS_CANCELED: 'canceled'}.get(res.status, 'aborted')
        reason = self._nav.get('reason') or ''
        if state == 'aborted':
            reason = f'Nav2 のエラー {res.result.error_code}'
        self._nav = self._nav_state(state, goal, reason, path=self._nav.get('path'),
                                    length=self._nav.get('path_m'))
        if self._mode == 'auto':
            self._nav_hold = NAV_END[state]
        self.get_logger().info(f'行き先: {NAV_LABELS[state]} {reason}')

    def _nav_cancel(self, reason):
        """向かっている行き先・考え中の道をやめる。reason が None なら新しい行き先に替えるだけ。"""
        self._nav_hold = None
        if self._nav['state'] in ('planning', 'sending', 'active'):
            self._nav_seq += 1
            if self._nav_handle is not None:
                self._nav_handle.cancel_goal_async()
            self._nav_handle = None
            if reason is not None:
                self._nav = self._nav_state('canceled', self._nav['goal'], reason,
                                            path=self._nav.get('path'))
                if self._mode == 'auto':
                    self._nav_hold = 'nav_stop'
                self.get_logger().info(f'行き先をやめる: {reason}')
        elif reason is not None:
            self._nav = self._nav_state('idle')

    # --- まだ行っていない所のはしへ（Nav2） ---
    def _grid_cb(self, msg: OccupancyGrid):
        info = msg.info
        grid = np.asarray(msg.data, dtype=np.int8).reshape(info.height, info.width)
        with self._lock:
            self._grid = (grid, info.resolution, info.origin.position.x, info.origin.position.y)

    def _map_pose(self, pose):
        """/odom の (x, y, yaw) → map。map → odom がまだ分からなければ None。"""
        mo = self._map_odom
        if pose is None or mo is None:
            return None
        c, sn = math.cos(mo[2]), math.sin(mo[2])
        return (mo[0] + c * pose[0] - sn * pose[1], mo[1] + sn * pose[0] + c * pose[1],
                _wrap(pose[2] + mo[2]))

    def _find_frontier(self, mp, now):
        """地図の「空き」と「不明」のさかいのかたまりのうち、えらべる中でいちばん近い所 (x, y)。なければ None。"""
        with self._lock:
            g = self._grid
        if g is None:
            return None
        grid, res, ox, oy = g
        h, w = grid.shape
        free, unknown, occ = grid == 0, grid < 0, grid >= 65
        edge = np.zeros_like(free)
        edge[1:, :] |= unknown[:-1, :]
        edge[:-1, :] |= unknown[1:, :]
        edge[:, 1:] |= unknown[:, :-1]
        edge[:, :-1] |= unknown[:, 1:]
        # 壁から frontier_clear_m 以内は行き先にしない
        r = max(1, int(math.ceil(self.frontier_clear / res)))
        near_wall = np.zeros_like(occ)
        oy_i, ox_i = np.nonzero(occ)
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                yy, xx = oy_i + dy, ox_i + dx
                ok = (yy >= 0) & (yy < h) & (xx >= 0) & (xx < w)
                near_wall[yy[ok], xx[ok]] = True
        # 車が行った所（/odom のマスのまん中を map にしたもの）の frontier_visited_m 以内
        visited = np.zeros_like(free)
        rv = int(math.ceil(self.frontier_visited / res))
        yy, xx = np.mgrid[-rv:rv + 1, -rv:rv + 1]
        disk = yy * yy + xx * xx <= rv * rv
        for (vx, vy) in list(self._visits):
            m = self._map_pose(((vx + 0.5) * self.explore_cell, (vy + 0.5) * self.explore_cell, 0.0))
            if m is None:
                continue
            ci, ri = int((m[0] - ox) / res), int((m[1] - oy) / res)
            r0, r1, c0, c1 = max(ri - rv, 0), min(ri + rv + 1, h), max(ci - rv, 0), min(ci + rv + 1, w)
            if r0 < r1 and c0 < c1:
                visited[r0:r1, c0:c1] |= disk[r0 - ri + rv:r1 - ri + rv, c0 - ci + rv:c1 - ci + rv]
        ok = free & ~near_wall
        # 車の所から、壁から frontier_clear_m 以上はなれた空きだけを通って行ける所（行けないすき間の向こうは選ばない）
        reach = _reachable(ok, int((mp[1] - oy) / res), int((mp[0] - ox) / res),
                           int(math.ceil(0.4 / res)))
        if reach is None:
            return None
        ok &= reach
        targets = (_clusters(ok & edge, self.frontier_min_cells)
                   + _clusters(ok & ~visited, self.frontier_min_area))
        bad = [(bx, by) for bx, by, t in self._frontier_bad if now - t < self.frontier_retry]
        best, best_d = None, float('inf')
        for cy, cx in targets:
            x, y = ox + (cx + 0.5) * res, oy + (cy + 0.5) * res
            d = math.hypot(x - mp[0], y - mp[1])
            if (d < self.frontier_min_dist or d >= best_d
                    or any(math.hypot(x - bx, y - by) < 0.6 for bx, by in bad)
                    or any(math.hypot(x - mx, y - my) < self.cliff_avoid + 0.2
                           for mx, my in self._cliff_marks)):
                continue
            best, best_d = (x, y), d
        return best

    def _frontier_start(self, now, pose) -> bool:
        """新しいマスに frontier_stale 秒入っていなければ、いちばん近い「はし」へ Nav2 で向かいはじめる。"""
        if (self.nav_client is None or self._frontier is not None
                or self._nav['state'] in ('planning', 'sending', 'active')
                or now - self._new_cell_at < self.frontier_stale or now - self._frontier_tried < 10.0):
            return False
        self._frontier_tried = now
        mp = self._map_pose(pose)
        goal = self._find_frontier(mp, now) if mp is not None else None
        if goal is None:
            return False
        yaw = math.degrees(math.atan2(goal[1] - mp[1], goal[0] - mp[0]))
        self._nav_goal_cb(String(data=json.dumps(
            {'action': 'go', 'x': goal[0], 'y': goal[1], 'yaw_deg': yaw})))
        self._frontier = {'x': round(goal[0], 2), 'y': round(goal[1], 2), 'since': now}
        self._frontier_count += 1
        self.get_logger().info(
            f'まだ行っていない所のはしへ: x {goal[0]:.2f} y {goal[1]:.2f}'
            f'（{math.hypot(goal[0] - mp[0], goal[1] - mp[1]):.1f} m 先）')
        return True

    def _frontier_watch(self, now, pose):
        """「はし」へ向かっているあいだ。終わった・時間切れ・段差の印に近づいたら、ふつうの走行にもどる。"""
        f = self._frontier
        state = self._nav['state']
        reason = None
        if state not in ('sending', 'active'):
            reason = None if state == 'succeeded' else (self._nav.get('reason') or state)
        else:
            mp = self._map_pose(pose)
            if now - f['since'] > self.frontier_timeout:
                reason = f'{self.frontier_timeout:.0f} 秒で着かない'
            elif mp is not None and any(math.hypot(mp[0] - mx, mp[1] - my) < self.cliff_avoid
                                        for mx, my in self._cliff_marks):
                reason = '段差の印に近づいた'
            else:
                return
            self._nav_cancel(reason)
        if reason is not None:
            self._frontier_bad.append((f['x'], f['y'], now))
        self.get_logger().info(f"はし（x {f['x']} y {f['y']}）: {'着いた' if reason is None else reason}")
        self._frontier = None
        self._nav_hold = None
        self._new_cell_at = now

    def _nav_step(self, now, mode):
        """Level 2 の速度。行き先がないときは None（Level 1 の走行）。"""
        state = self._nav['state']
        if self._prev_mode == 'auto' and mode != 'auto':
            self._nav_hold = None
            if state in ('sending', 'active'):
                self._nav_cancel('自動を抜けた')
        self._prev_mode = mode
        state = self._nav['state']
        if (mode != 'auto' and state in ('sending', 'active')
                and now - self._nav['since'] > self.nav_auto_wait):
            self._nav_cancel(f'{self.nav_auto_wait:.0f} 秒たっても自動に入らない')
        if mode != 'auto':
            return None
        if self._nav['state'] in ('sending', 'active'):
            if self._unstick is None and self._escape is None:
                self._set_behavior('nav')
            with self._lock:
                fresh = now - self._nav_cmd_stamp <= self.nav_cmd_timeout
                if not fresh:
                    return Twist()
                cmd = Twist()
                cmd.linear.x = self._nav_cmd.linear.x
                cmd.linear.y = self._nav_cmd.linear.y
                cmd.angular.z = max(-self.nav_max_angular,
                                    min(self.nav_max_angular, self._nav_cmd.angular.z))
                return cmd
        if self._nav_hold:
            self._set_behavior(self._nav_hold)
            return Twist()
        return None

    # --- 行動決定 ---
    def _set_behavior(self, behavior: str):
        if behavior != self._behavior:
            self.get_logger().info(f'行動: {LABELS.get(self._behavior)} → {LABELS.get(behavior)}')
            self._behavior = behavior
            self._behavior_since = time.monotonic()

    def _decide(self) -> Twist:
        now = time.monotonic()
        with self._lock:
            mode = self._mode
            s = self._sectors
            scan_age = now - self._scan_stamp if self._scan_stamp else None
            cliff = self._cliff if now - self._cliff_stamp <= self.cliff_timeout else None
            pose = self._pose if now - self._pose_stamp <= self.odom_timeout else None
            tilt = self._tilt if now - self._tilt_stamp <= 1.0 else None
        cmd = Twist()
        nav_cmd = self._nav_step(now, mode)
        if mode != 'auto':
            self._frontier = None
            self._new_cell_at = now
            self._escape = None
            self._unstick = None
            self._unstick_times = []
            self._tilt_base = None
            self._tilt_since = None
            self._set_behavior('standby')
            return cmd
        if s is None or scan_age is None or scan_age > self.scan_max_age:
            self._set_behavior('blocked')
            return cmd

        cliff_front = bool(cliff and cliff.get('block_forward'))
        cliff_back = bool(cliff and cliff.get('block_backward'))
        if self._behavior in ('stuck', 'tilted'):
            return cmd
        if self._tilted(now, tilt):
            self._escape = None
            self._unstick = None
            self._set_behavior('tilted')
            # 横の段差は見えないので、落ちた所（車のまん中）も段差の印にする
            if pose is not None:
                self._add_cliff(pose, ahead=0.0)
            self.get_logger().warn('かたむいた（落ちた・乗り上げた）ので止まる。ここを段差の印にした')
            return cmd
        if self._unstick is not None:
            un_cmd = self._unstick_step(now, s, pose, cliff_back)
            if un_cmd is not None:
                return un_cmd
        if cliff_front and (self._escape is None or self._escape['phase'] == 'turn'):
            self._escape = {'phase': 'back', 'since': now, 'start': pose}
            if pose is not None:
                self._add_cliff(pose)
            self.get_logger().info('前に段差: 下がって回る')
            if self._frontier is not None:
                # Nav2 は段差の印を知らないので、その行き先はやめる（frontier_retry_sec えらばない）
                f = self._frontier
                self._nav_cancel('前に段差')
                self._frontier_bad.append((f['x'], f['y'], now))
                self._frontier = None
                self._nav_hold = None
                self._new_cell_at = now
        if self._escape is not None:
            esc_cmd = self._escape_step(now, s, pose, cliff_front, cliff_back)
            if esc_cmd is not None:
                return esc_cmd
        if self._frontier is not None:
            self._frontier_watch(now, pose)
        if nav_cmd is not None:
            return nav_cmd
        if self._frontier_start(now, pose):
            return cmd

        front = s.get('front')
        path_v = s.get('path') if s.get('path') is not None else float('inf')
        # 覚えている段差は、前の障害物と同じにみる（その手前で曲がる）
        path_v = min(path_v, self._cliff_ahead(pose))
        left = s.get('left')
        right = s.get('right')
        back = s.get('back')
        big = float('inf')
        front_v = front if front is not None else big
        left_v = left if left is not None else big
        right_v = right if right is not None else big
        back_v = back if back is not None else big

        in_turn = self._behavior == 'turn'
        turning_long_enough = now - self._behavior_since >= self.min_turn + self._turn_extra

        if (front_v <= self.backoff_d and left_v <= self.backoff_d
                and right_v <= self.backoff_d and back is not None
                and back_v > self.stop_d):
            self._set_behavior('backoff')
            cmd.linear.x = -self.backoff_speed
            return cmd

        self._count_visit(pose)
        if in_turn:
            if self._spun_out(pose):
                self._set_behavior('stuck')
                self.get_logger().warn(
                    f'{math.degrees(self.turn_max):.0f}° 回っても出口がないので止まる')
                return cmd
            left_rad = self._target_left(pose)
            if left_rad is not None and now - self._behavior_since < self.explore_turn_timeout:
                # 曲がる先の向きまで回る（そこまで回って、前がまだふさがっていれば回りつづける）
                if left_rad > self.explore_reach or path_v <= self.stop_d:
                    cmd.angular.z = self._turn_dir * self.turn
                    return cmd
                self._target_yaw = None
                self._start_via(now)
            elif path_v < self.clear_d or not turning_long_enough:
                cmd.angular.z = self._turn_dir * self.turn
                return cmd
            self._target_yaw = None
            self._via_pending = False

        if path_v <= self.stop_d:
            self._via_pending, self._via_until = False, 0.0
            # 空いている側へ旋回（左が広ければ左旋回 = +z）。減速中に曲がりはじめた向きがあればそのまま
            self._turn_dir = self._pre_dir if self._pre_dir is not None else self._pick_turn(s, pose)
            self._pre_dir = None
            self._turn_extra = random.uniform(0.0, self.turn_random)
            self._turn_acc, self._turn_yaw = 0.0, None
            self._set_behavior('turn')
            cmd.angular.z = self._turn_dir * self.turn
            return cmd

        if path_v <= self.slow_d + (self.slow_hyst if self._behavior == 'slow' else 0.0):
            # 通路の入口へ向かっているあいだは、向きを選びなおさずに入口の手前までまっすぐ進む
            via = now < self._via_until
            if not via and (self._pre_dir is None or (self._behavior != 'slow'
                                                      and now - self._slow_left > self.slow_keep)):
                self._pre_dir = self._pick_turn(s, pose)
            self._set_behavior('slow')
            cmd.linear.x = self.slow
            # 止まる距離に近づくほど強く、曲がる先の側へ曲がる（向きは減速に入ったときに 1 回だけ決める）
            left_rad = None if via else self._target_left(pose)
            if not via:
                self._turn_dir = self._pre_dir
                if left_rad is not None and left_rad <= self.explore_reach:
                    self._start_via(now)
            if not via and (left_rad is None or left_rad > self.explore_reach):
                near = (self.slow_d - path_v) / max(self.slow_d - self.stop_d, 1e-3)
                cmd.angular.z = self._pre_dir * self.pre_turn_gain * self.turn * max(0.0, min(1.0, near))
        else:
            if self._behavior == 'slow':
                self._slow_left = now
            if now - self._slow_left > self.slow_keep:
                self._pre_dir = None
                self._target_yaw = None
            self._set_behavior('forward')
            cmd.linear.x = self.cruise
        if cmd.angular.z == 0.0:
            vy = self._center_vy(s)
            # 横には段差センサーがないので、覚えている段差のほうへは横にずれない
            if vy != 0.0 and pose is not None and self._toward_cliff(pose, math.copysign(math.pi / 2, vy), 0.2):
                vy = 0.0
            cmd.linear.y = vy
        return cmd

    def _open_side(self, s) -> float:
        """曲がる向き（左 +1 / 右 -1）。両方とも random_side_distance より空いている・左右の差が 0.1 m 未満ならランダム、ちがえば空いているほう。"""
        big = float('inf')
        left = min(v for v in (s.get('left'), s.get('front_left'), big) if v is not None)
        right = min(v for v in (s.get('right'), s.get('front_right'), big) if v is not None)
        if (left >= self.random_side and right >= self.random_side) or abs(left - right) < 0.1:
            return random.choice((1.0, -1.0))
        return 1.0 if left >= right else -1.0

    def _choose_heading(self, pose):
        """曲がる先の向き（今の前からの角度 rad、左 +）。候補がなければ None。
        通路の入口を通る向きを選んだときは self._via_pick にその (s, th2, d2) を入れる（ほかは None）。"""
        with self._lock:
            free = list(self._free)
            pts = self._scan_xy
        cands, weights, vias = [], [], []
        for th, d in free:
            if abs(th) < self.explore_min_turn:
                continue
            via = None
            if d >= self.explore_min_free:
                reach = min(d, 3.0)
            elif self.explore_via:
                via = self._via(pts, th, d)
                if via is None:
                    continue
                reach = via[0] + min(via[2], 3.0)
            else:
                continue
            novelty = 1.0
            if pose is not None:
                yaw = pose[2] + th
                if via is None:
                    if self._toward_cliff(pose, th, reach):
                        continue
                    # 行ける所まで（前の物の stop_distance 手前まで）のマスだけ数える。壁ぎわ・行き止まりのマスは車が入れない
                    legs = [(pose[0], pose[1], yaw, max(reach - self.stop_d, self.explore_cell))]
                else:
                    sv, th2, d2 = via
                    mid = (pose[0] + sv * math.cos(yaw), pose[1] + sv * math.sin(yaw), pose[2])
                    if self._toward_cliff(pose, th, sv) or self._toward_cliff(mid, th2, min(d2, 3.0)):
                        continue
                    legs = [(pose[0], pose[1], yaw, sv),
                            (mid[0], mid[1], pose[2] + th2, max(min(d2, 3.0) - self.stop_d, self.explore_cell))]
                novelty = self._novelty(legs)
            cands.append(th)
            weights.append(reach * novelty * novelty)
            vias.append(via)
        if not cands:
            self._via_pick = None
            return None
        k = random.choices(range(len(cands)), weights=weights)[0]
        self._via_pick = vias[k]
        return cands[k]

    def _start_via(self, now):
        """通路の入口を通る向きまで回り終わったら、explore_via_sec 秒は入口の手前までまっすぐ進む。"""
        if self._via_pending:
            self._via_until = now + self.explore_via_sec
            self._via_pending = False

    def _via(self, pts, th, d):
        """th の向きに、通路の入口の手前（d - stop_distance）まで進んだ所から、±explore_via_deg のうち
        車の幅でいちばん遠くまで進める向き。(進む m, その向き rad（今の前から）, 進める m) か None。"""
        sv = d - self.stop_d
        if sv < self.explore_via_min or len(pts) == 0:
            return None
        ths = th + self._via_dth
        xy = pts - np.array([sv * math.cos(th), sv * math.sin(th)])
        along = xy[:, :1] * np.cos(ths) + xy[:, 1:] * np.sin(ths)
        perp = xy[:, 1:] * np.cos(ths) - xy[:, :1] * np.sin(ths)
        hit = np.where((along > 0.0) & (np.abs(perp) <= self.path_half), along, 4.0).min(axis=0)
        k = int(np.argmax(hit))
        if hit[k] < self.explore_min_free:
            return None
        return sv, float(ths[k]), float(min(hit[k], 4.0))

    def _novelty(self, legs) -> float:
        """legs = [(x, y, yaw, 長さ m)] の線の上の explore_cell ごとのマスが、どれだけ「まだ行っていない」か（0〜1）。"""
        total, n = 0.0, 0
        for x, y, yaw, length in legs:
            steps = max(1, int(length / self.explore_cell))
            for k in range(1, steps + 1):
                r = k * length / steps
                cell = (math.floor((x + r * math.cos(yaw)) / self.explore_cell),
                        math.floor((y + r * math.sin(yaw)) / self.explore_cell))
                total += 1.0 / (1.0 + self._visits.get(cell, 0))
                n += 1
        return total / n if n else 1.0

    def _toward_cliff(self, pose, th, reach) -> bool:
        """その向きに reach 進むあいだに、覚えている段差の cliff_avoid 以内を通るか。"""
        yaw = pose[2] + th
        c, sn = math.cos(yaw), math.sin(yaw)
        for cx, cy in self._known_cliffs():
            dx, dy = cx - pose[0], cy - pose[1]
            along = dx * c + dy * sn
            if -0.2 < along < reach + 0.3 and abs(dy * c - dx * sn) < self.cliff_avoid:
                return True
        return False

    def _pick_turn(self, s, pose) -> float:
        """曲がる向きを決める（左 +1 / 右 -1）。/odom があれば曲がる先の向き _target_yaw も決める。"""
        th = self._choose_heading(pose)
        self._via_pending = self._via_pick is not None
        self._via_until = 0.0
        if th is None:
            self._target_yaw = None
            return self._open_side(s)
        self._target_yaw = _wrap(pose[2] + th) if pose is not None else None
        return 1.0 if th > 0 else -1.0

    def _target_left(self, pose):
        """曲がる先まであと何 rad 回るか（_turn_dir の向きに +）。わからなければ None。"""
        if self._target_yaw is None or pose is None:
            return None
        return _wrap(self._target_yaw - pose[2]) * self._turn_dir

    def _count_visit(self, pose):
        if pose is None:
            return
        cell = (math.floor(pose[0] / self.explore_cell), math.floor(pose[1] / self.explore_cell))
        if cell != self._last_cell:
            if cell not in self._visits:
                self._new_cell_at = time.monotonic()
            self._visits[cell] = self._visits.get(cell, 0) + 1
            self._last_cell = cell

    def _watch_progress(self, now, cmd: Twist):
        """動く指令なのに距離も向きも変わらないあいだが stuck_sec 秒（Nav2 なら nav_stuck_sec 秒）つづいたら、抜け出しを始める。"""
        moving = (abs(cmd.linear.x) > 0.01 or abs(cmd.linear.y) > 0.01 or abs(cmd.angular.z) > 0.05)
        with self._lock:
            s = self._sectors
            pose = self._pose if now - self._pose_stamp <= self.odom_timeout else None
        nav = self._behavior.startswith('nav')
        if (not moving or s is None or self._unstick is not None or self._behavior == 'stuck'
                or (nav and pose is None)):
            self._progress = None
            return
        keys = ('front', 'back', 'left', 'right', 'path')
        snap = {k: s.get(k) for k in keys}
        yaw = pose[2] if pose else None
        ref = self._progress
        if ref is None or ref.get('nav') != nav:
            self._progress = {'since': now, 'snap': snap, 'yaw': yaw, 'pose': pose, 'nav': nav}
            return
        if now - ref['since'] < (self.nav_stuck_sec if nav else self.stuck_sec):
            return
        if nav:
            # Nav2 はゆっくり近づくので、LiDAR のゆらぎでなく、車輪で数えた位置で見る
            moved = math.hypot(pose[0] - ref['pose'][0], pose[1] - ref['pose'][1])
        else:
            moved = max([abs(snap[k] - ref['snap'][k]) for k in keys
                         if snap[k] is not None and ref['snap'][k] is not None] or [float('inf')])
        turned = abs(_wrap(yaw - ref['yaw'])) if yaw is not None and ref['yaw'] is not None else 0.0
        self._progress = {'since': now, 'snap': snap, 'yaw': yaw, 'pose': pose, 'nav': nav}
        if moved >= self.stuck_move or turned >= self.stuck_turn:
            return
        # 抜け出しのあと unstick_window 秒、動けないことがなければ数えなおす（10 秒おきに動けないときも数がたまるように）
        if self._unstick_times and now - self._unstick_times[-1] > self.unstick_window:
            self._unstick_times = []
        if len(self._unstick_times) >= self.unstick_max:
            self._escape = None
            if nav:
                self._nav_give_up()
                return
            self._set_behavior('stuck')
            self.get_logger().warn(f'動けない: 抜け出しを {self.unstick_max} 回しても動けないので止まる')
            return
        self._unstick_times.append(now)
        self._escape = None
        n = len(self._unstick_times)
        back_dir = -1.0 if cmd.linear.x <= 0.0 and abs(cmd.linear.x) > 0.01 else 1.0
        if nav:
            # Nav2 のときは、1 回に 1 つだけ。回ごとに方法を変える（1 回目 下がる・2 回目 横・3 回目 反対へ回る）
            k = (n - 1) % len(UNSTICK_PHASES)
            phases = list(UNSTICK_PHASES[k:] + UNSTICK_PHASES[:k])
            turn_dir = (-math.copysign(1.0, cmd.angular.z) if abs(cmd.angular.z) > 0.05
                        else self._open_side(s))
        else:
            phases = list(UNSTICK_PHASES)
            turn_dir = -self._turn_dir if abs(cmd.angular.z) > 0.05 else self._open_side(s)
        self._unstick = {'phases': phases, 'single': nav, 'acted': False, 'since': now, 'back_dir': back_dir,
                         'turn_dir': turn_dir, 'last_yaw': yaw, 'turned': 0.0}
        self.get_logger().info(
            f'動けない（{(self.nav_stuck_sec if nav else self.stuck_sec):.0f} 秒で {moved:.2f} m・'
            f'{math.degrees(turned):.0f}°）: 抜け出し {n} 回目（{UNSTICK_NAMES[phases[0]]}から）')

    def _nav_give_up(self):
        """Nav2 で向かっているのに、抜け出しを unstick_max 回しても動けない。その行き先はやめる（パトロールは次へ）。"""
        reason = f'抜け出しを {self.unstick_max} 回しても動けない'
        goal, path = self._nav.get('goal'), self._nav.get('path')
        self._nav_cancel(None)
        self._nav = self._nav_state('aborted', goal, reason, path=path)
        self._nav['stuck'] = True
        self._nav_hold = NAV_END['aborted']
        self._unstick_times = []
        self._progress = None
        self.get_logger().warn(f'行き先: {reason}')

    def _unstick_step(self, now, s, pose, cliff_back):
        """抜け出しを phases の順に（できないものはとばす）。single（Nav2）なら、できた 1 つで終わる。終わったら None。"""
        un = self._unstick
        while un['phases']:
            cmd = getattr(self, '_unstick_' + un['phases'][0])(now, un, s, pose, cliff_back)
            if cmd is not None:
                un['acted'] = True
                return cmd
            un['phases'] = un['phases'][1:]
            un.update(since=now, last_yaw=pose[2] if pose else None, turned=0.0)
            if un['single'] and un['acted']:
                break
        self._unstick = None
        self._progress = None
        self._turn_dir = un['turn_dir']
        return None

    def _unstick_back(self, now, un, s, pose, cliff_back):
        # 前に進もうとして動けなかったら下がる。下がろうとして動けなかったら前へ
        go = -un['back_dir']
        room = (s.get('back') if go < 0 else s.get('path'))
        blocked = ((go < 0 and cliff_back) or (room is not None and room <= self.backoff_d)
                   or (pose is not None and self._toward_cliff(pose, 0.0 if go > 0 else math.pi, 0.15)))
        if blocked or now - un['since'] >= self.unstick_back_sec:
            return None
        self._set_behavior('stuck_back')
        cmd = Twist()
        cmd.linear.x = go * self.backoff_speed
        return cmd

    def _unstick_side(self, now, un, s, pose, cliff_back):
        big = float('inf')
        left = s.get('side_left') if s.get('side_left') is not None else big
        right = s.get('side_right') if s.get('side_right') is not None else big
        if pose is not None:
            if self._toward_cliff(pose, math.pi / 2, 0.2):
                left = 0.0
            if self._toward_cliff(pose, -math.pi / 2, 0.2):
                right = 0.0
        if max(left, right) <= 0.25 or now - un['since'] >= self.unstick_side_sec:
            return None
        self._set_behavior('stuck_side')
        cmd = Twist()
        cmd.linear.y = (1.0 if left >= right else -1.0) * self.backoff_speed
        return cmd

    def _unstick_turn(self, now, un, s, pose, cliff_back):
        if pose is not None and un.get('last_yaw') is not None:
            un['turned'] += abs(_wrap(pose[2] - un['last_yaw']))
            un['last_yaw'] = pose[2]
        if un['turned'] >= self.unstick_turn or now - un['since'] >= self.unstick_turn_sec:
            return None
        self._set_behavior('stuck_turn')
        cmd = Twist()
        cmd.angular.z = un['turn_dir'] * self.turn
        return cmd

    def _center_vy(self, s) -> float:
        """左右の両方が近いとき、まん中へ寄る横の速さ（左 +）。向きは変えない（回るとお尻が振れるため）。"""
        sl, sr = s.get('side_left'), s.get('side_right')
        if sl is None or sr is None or sl > self.center_range or sr > self.center_range:
            return 0.0
        off = (sl - sr) / 2.0   # + なら左のほうが空いている
        if abs(off) <= self.center_deadband:
            return 0.0
        vy = self.center_gain * off
        return max(-self.center_speed, min(self.center_speed, vy))

    def _escape_step(self, now, s, pose, cliff_front, cliff_back):
        """段差から下がって回る。終わったら None（ふつうの走行にもどる）。"""
        esc = self._escape
        cmd = Twist()
        if now - esc['since'] > self.cliff_escape_timeout:
            if cliff_front:
                self._set_behavior('cliff_stuck')
                return cmd
            self._escape = None
            return None
        if pose is None:
            self._set_behavior('cliff_stuck')
            return cmd
        if esc['start'] is None:
            esc['start'] = pose
        if esc['phase'] == 'back':
            moved = math.hypot(pose[0] - esc['start'][0], pose[1] - esc['start'][1])
            if cliff_back or (moved >= self.cliff_backoff_m and not cliff_front):
                if cliff_front:
                    self._set_behavior('cliff_stuck')
                    return cmd
                big = float('inf')
                left = min(v for v in (s.get('left'), s.get('front_left'), big) if v is not None)
                right = min(v for v in (s.get('right'), s.get('front_right'), big) if v is not None)
                self._turn_dir = 1.0 if left >= right else -1.0
                esc.update(phase='turn', since=now, last_yaw=pose[2], turned=0.0)
            else:
                self._set_behavior('cliff_back')
                cmd.linear.x = -self.backoff_speed
                return cmd
        esc['turned'] += abs(_wrap(pose[2] - esc['last_yaw']))
        esc['last_yaw'] = pose[2]
        if esc['turned'] >= self.cliff_turn:
            self._escape = None
            return None
        self._set_behavior('cliff_turn')
        cmd.angular.z = self._turn_dir * self.turn
        return cmd

    def _timer_cb(self):
        cmd = self._decide()
        self._watch_progress(time.monotonic(), cmd)
        with self._lock:
            self._last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def _status_cb(self):
        now = time.monotonic()
        with self._lock:
            payload = {
                'stamp': round(time.time(), 3),
                'level': 2 if self._behavior.startswith('nav') else 1,
                'method': ('Nav2（地図の行き先へ）' if self._behavior.startswith('nav')
                           else 'LiDAR 反応型（地図なし）'),
                'mode': self._mode,
                'behavior': self._behavior,
                'behavior_label': LABELS.get(self._behavior, self._behavior),
                'behavior_s': round(now - self._behavior_since, 1),
                'turn_dir': 'left' if self._turn_dir > 0 else 'right',
                'explore_target_deg': (round(math.degrees(self._target_yaw), 1)
                                       if self._target_yaw is not None else None),
                'explore_via': time.monotonic() < self._via_until,
                'visited_cells': len(self._visits),
                'frontier': ({'x': self._frontier['x'], 'y': self._frontier['y']}
                             if self._frontier else None),
                'frontier_goals': self._frontier_count,
                'cliff_points': len(self._cliff_pts) + len(self._cliff_marks),
                'cliff_marks': [[round(x, 2), round(y, 2)] for x, y in self._cliff_marks],
                'tilt_deg': (round(math.degrees(max(abs(_wrap(self._tilt[0] - self._tilt_base[0])),
                                                    abs(self._tilt[1] - self._tilt_base[1]))), 1)
                             if self._tilt is not None and self._tilt_base is not None else None),
                'unstick': ({'phase': (self._unstick['phases'] or [None])[0], 'count': len(self._unstick_times)}
                            if self._unstick else None),
                'cliff_turned_deg': (round(math.degrees(self._escape.get('turned', 0.0)), 1)
                                     if self._escape else None),
                'sectors': {k: (round(v, 3) if v is not None else None)
                            for k, v in (self._sectors or {}).items()},
                'scan_age_s': (round(now - self._scan_stamp, 2)
                               if self._scan_stamp else None),
                'cmd': {
                    'linear_x': round(self._last_cmd.linear.x, 3),
                    'linear_y': round(self._last_cmd.linear.y, 3),
                    'angular_z': round(self._last_cmd.angular.z, 3),
                },
                'nav': {
                    **{k: v for k, v in self._nav.items() if k != 'since'},
                    'label': NAV_LABELS.get(self._nav['state'], self._nav['state']),
                    'age_s': round(now - self._nav['since'], 1),
                    'available': bool(self.nav_client and self.nav_client.server_is_ready()),
                },
                'params': {
                    'cruise_speed': self.cruise, 'slow_speed': self.slow,
                    'turn_speed': self.turn, 'stop_distance': self.stop_d,
                    'slow_distance': self.slow_d, 'clear_distance': self.clear_d,
                    'cliff_backoff_m': self.cliff_backoff_m,
                    'cliff_turn_deg': round(math.degrees(self.cliff_turn), 1),
                },
            }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyNode()
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
