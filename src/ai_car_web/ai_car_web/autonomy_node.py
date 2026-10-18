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
  blocked   LiDAR 未受信などで停止
  nav       Nav2 で地図の行き先へ向かう（/cmd_vel_nav を中継）
  nav_done  行き先に着いた（ゼロ速度。AUTO を抜けるか次の行き先まで）
  nav_failed  Nav2 が行き先へ行けなかった（ゼロ速度）
  nav_stop  行き先をやめた（ゼロ速度）
"""

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

LABELS = {
    'standby': '待機', 'forward': '前進', 'slow': '減速', 'turn': '旋回',
    'backoff': '後退', 'blocked': '停止（センサー待ち）',
    'cliff_back': '段差から後退', 'cliff_turn': '段差から旋回',
    'cliff_stuck': '停止（段差で動けない）',
    'stuck_back': '抜け出し（後退）', 'stuck_side': '抜け出し（横）', 'stuck_turn': '抜け出し（旋回）',
    'stuck': '停止（動けない）',
    'nav': '行き先へ', 'nav_done': '行き先に着いた', 'nav_failed': '停止（行き先へ行けない）',
    'nav_stop': '停止（行き先をやめた）',
}
NAV_LABELS = {
    'idle': 'なし', 'unavailable': 'Nav2 が動いていない', 'planning': '道を考え中',
    'planned': '道あり（まだ動かない）', 'no_path': '道が見つからない', 'sending': '送り中',
    'active': '向かっている', 'succeeded': '着いた', 'aborted': '行けなかった', 'canceled': 'やめた',
}
NAV_END = {'succeeded': 'nav_done', 'aborted': 'nav_failed', 'canceled': 'nav_stop'}
MAX_PATH_POINTS = 120


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _path_points(path: Path):
    """道を MAX_PATH_POINTS 点までに間引いた [[x, y], ...] と長さ [m]。"""
    pts = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    step = max(1, math.ceil(len(pts) / MAX_PATH_POINTS))
    keep = pts[::step]
    if pts and keep[-1] != pts[-1]:
        keep.append(pts[-1])
    return [[round(x, 2), round(y, 2)] for x, y in keep], length


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
        self.declare_parameter('stuck_move_m', 0.04)
        self.declare_parameter('stuck_turn_deg', 10.0)
        self.declare_parameter('unstick_back_sec', 1.5)
        self.declare_parameter('unstick_side_sec', 1.5)
        self.declare_parameter('unstick_turn_deg', 60.0)
        self.declare_parameter('unstick_turn_sec', 4.0)
        # この回数抜け出しても（unstick_window 秒のうちに）また動けなければ止まる
        self.declare_parameter('unstick_max', 3)
        self.declare_parameter('unstick_window', 30.0)
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
        self.declare_parameter('nav_auto_wait', 5.0)

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
        self.path_half = float(g('path_half_width').value)
        self.center_range = float(g('center_range').value)
        self.center_gain = float(g('center_gain').value)
        self.center_speed = float(g('center_speed').value)
        self.center_deadband = float(g('center_deadband').value)
        self.pre_turn_gain = float(g('pre_turn_gain').value)
        self.stuck_sec = float(g('stuck_sec').value)
        self.stuck_move = float(g('stuck_move_m').value)
        self.stuck_turn = math.radians(float(g('stuck_turn_deg').value))
        self.unstick_back_sec = float(g('unstick_back_sec').value)
        self.unstick_side_sec = float(g('unstick_side_sec').value)
        self.unstick_turn = math.radians(float(g('unstick_turn_deg').value))
        self.unstick_turn_sec = float(g('unstick_turn_sec').value)
        self.unstick_max = int(g('unstick_max').value)
        self.unstick_window = float(g('unstick_window').value)
        self.cliff_timeout = float(g('cliff_timeout').value)
        self.odom_timeout = float(g('odom_timeout').value)
        self.cliff_backoff_m = float(g('cliff_backoff_m').value)
        self.cliff_turn = math.radians(float(g('cliff_turn_deg').value))
        self.cliff_escape_timeout = float(g('cliff_escape_timeout').value)
        self.nav_frame = g('nav_frame').value
        self.nav_cmd_timeout = float(g('nav_cmd_timeout').value)
        self.nav_auto_wait = float(g('nav_auto_wait').value)

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
        rate = float(g('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        self.create_timer(0.2, self._status_cb)

    # --- 入力 ---
    def _scan_cb(self, msg: LaserScan):
        sectors = {'front': [], 'left': [], 'right': [], 'back': []}
        path = []
        side_l, side_r = [], []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + self.offset
            a = math.atan2(math.sin(a), math.cos(a))
            x, y = r * math.cos(a), r * math.sin(a)
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
        with self._lock:
            self._sectors = summary
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
            self._set_behavior('nav')
            with self._lock:
                fresh = now - self._nav_cmd_stamp <= self.nav_cmd_timeout
                return self._nav_cmd if fresh else Twist()
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
        cmd = Twist()
        nav_cmd = self._nav_step(now, mode)
        if mode != 'auto':
            self._escape = None
            self._unstick = None
            self._unstick_times = []
            self._set_behavior('standby')
            return cmd
        if s is None or scan_age is None or scan_age > self.scan_max_age:
            self._set_behavior('blocked')
            return cmd

        cliff_front = bool(cliff and cliff.get('block_forward'))
        cliff_back = bool(cliff and cliff.get('block_backward'))
        if self._behavior == 'stuck':
            return cmd
        if self._unstick is not None:
            un_cmd = self._unstick_step(now, s, pose, cliff_back)
            if un_cmd is not None:
                return un_cmd
        if cliff_front and (self._escape is None or self._escape['phase'] == 'turn'):
            self._escape = {'phase': 'back', 'since': now, 'start': pose}
            self.get_logger().info('前に段差: 下がって回る')
        if self._escape is not None:
            esc_cmd = self._escape_step(now, s, pose, cliff_front, cliff_back)
            if esc_cmd is not None:
                return esc_cmd
        if nav_cmd is not None:
            return nav_cmd

        front = s.get('front')
        path_v = s.get('path') if s.get('path') is not None else float('inf')
        left = s.get('left')
        right = s.get('right')
        back = s.get('back')
        big = float('inf')
        front_v = front if front is not None else big
        left_v = left if left is not None else big
        right_v = right if right is not None else big
        back_v = back if back is not None else big

        in_turn = self._behavior == 'turn'
        turning_long_enough = now - self._behavior_since >= self.min_turn

        if (front_v <= self.backoff_d and left_v <= self.backoff_d
                and right_v <= self.backoff_d and back is not None
                and back_v > self.stop_d):
            self._set_behavior('backoff')
            cmd.linear.x = -self.backoff_speed
            return cmd

        if in_turn and (path_v < self.clear_d or not turning_long_enough):
            cmd.angular.z = self._turn_dir * self.turn
            return cmd

        if path_v <= self.stop_d:
            # 空いている側へ旋回（左が広ければ左旋回 = +z）。減速中に曲がりはじめた向きがあればそのまま
            fl = s.get('front_left')
            fr = s.get('front_right')
            left_score = min(left_v, fl if fl is not None else big)
            right_score = min(right_v, fr if fr is not None else big)
            self._turn_dir = (self._pre_dir if self._pre_dir is not None
                              else 1.0 if left_score >= right_score else -1.0)
            self._pre_dir = None
            self._set_behavior('turn')
            cmd.angular.z = self._turn_dir * self.turn
            return cmd

        if path_v <= self.slow_d:
            if self._behavior != 'slow' or self._pre_dir is None:
                self._pre_dir = self._open_side(s)
            self._set_behavior('slow')
            cmd.linear.x = self.slow
            # 止まる距離に近づくほど強く、空いている側へ曲がる（向きは減速に入ったときに 1 回だけ決める）
            near = (self.slow_d - path_v) / max(self.slow_d - self.stop_d, 1e-3)
            cmd.angular.z = self._pre_dir * self.pre_turn_gain * self.turn * max(0.0, min(1.0, near))
        else:
            self._pre_dir = None
            self._set_behavior('forward')
            cmd.linear.x = self.cruise
        if cmd.angular.z == 0.0:
            cmd.linear.y = self._center_vy(s)
        return cmd

    @staticmethod
    def _open_side(s) -> float:
        """左右の空いているほう（左 +1 / 右 -1）。"""
        big = float('inf')
        left = min(v for v in (s.get('left'), s.get('front_left'), big) if v is not None)
        right = min(v for v in (s.get('right'), s.get('front_right'), big) if v is not None)
        return 1.0 if left >= right else -1.0

    def _watch_progress(self, now, cmd: Twist):
        """動く指令なのに距離も向きも変わらないあいだが stuck_sec 秒つづいたら、抜け出しを始める。"""
        moving = (abs(cmd.linear.x) > 0.01 or abs(cmd.linear.y) > 0.01 or abs(cmd.angular.z) > 0.05)
        with self._lock:
            s = self._sectors
            pose = self._pose if now - self._pose_stamp <= self.odom_timeout else None
        if (not moving or s is None or self._unstick is not None or self._behavior.startswith('nav')
                or self._behavior == 'stuck'):
            self._progress = None
            return
        keys = ('front', 'back', 'left', 'right', 'path')
        snap = {k: s.get(k) for k in keys}
        yaw = pose[2] if pose else None
        ref = self._progress
        if ref is None:
            self._progress = {'since': now, 'snap': snap, 'yaw': yaw}
            return
        if now - ref['since'] < self.stuck_sec:
            return
        moved = max([abs(snap[k] - ref['snap'][k]) for k in keys
                     if snap[k] is not None and ref['snap'][k] is not None] or [float('inf')])
        turned = abs(_wrap(yaw - ref['yaw'])) if yaw is not None and ref['yaw'] is not None else 0.0
        self._progress = {'since': now, 'snap': snap, 'yaw': yaw}
        if moved >= self.stuck_move or turned >= self.stuck_turn:
            return
        self._unstick_times = [t for t in self._unstick_times if now - t <= self.unstick_window]
        if len(self._unstick_times) >= self.unstick_max:
            self._escape = None
            self._set_behavior('stuck')
            self.get_logger().warn(f'動けない: 抜け出しを {self.unstick_max} 回しても動けないので止まる')
            return
        self._unstick_times.append(now)
        self._escape = None
        back_dir = -1.0 if cmd.linear.x <= 0.0 and abs(cmd.linear.x) > 0.01 else 1.0
        self._unstick = {'phase': 'back', 'since': now, 'back_dir': back_dir,
                         'turn_dir': -self._turn_dir if abs(cmd.angular.z) > 0.05 else self._open_side(s)}
        self.get_logger().info(
            f'動けない（{self.stuck_sec:.0f} 秒で {moved:.2f} m・{math.degrees(turned):.0f}°）: '
            f'抜け出し {len(self._unstick_times)} 回目')

    def _unstick_step(self, now, s, pose, cliff_back):
        """下がる → 横にずれる → 反対へ回る。終わったら None（ふつうの走行にもどる）。"""
        un = self._unstick
        cmd = Twist()
        big = float('inf')
        if un['phase'] == 'back':
            # 前に進もうとして動けなかったら下がる。下がろうとして動けなかったら前へ
            go = -un['back_dir']
            room = (s.get('back') if go < 0 else s.get('path'))
            blocked = (go < 0 and cliff_back) or (room is not None and room <= self.backoff_d)
            if not blocked and now - un['since'] < self.unstick_back_sec:
                self._set_behavior('stuck_back')
                cmd.linear.x = go * self.backoff_speed
                return cmd
            un.update(phase='side', since=now)
        if un['phase'] == 'side':
            left = s.get('side_left') if s.get('side_left') is not None else big
            right = s.get('side_right') if s.get('side_right') is not None else big
            side = 1.0 if left >= right else -1.0
            if max(left, right) > 0.25 and now - un['since'] < self.unstick_side_sec:
                self._set_behavior('stuck_side')
                cmd.linear.y = side * self.backoff_speed
                return cmd
            un.update(phase='turn', since=now, last_yaw=pose[2] if pose else None, turned=0.0)
        if pose is not None and un.get('last_yaw') is not None:
            un['turned'] += abs(_wrap(pose[2] - un['last_yaw']))
            un['last_yaw'] = pose[2]
        if un['turned'] >= self.unstick_turn or now - un['since'] >= self.unstick_turn_sec:
            self._unstick = None
            self._progress = None
            self._turn_dir = un['turn_dir']
            return None
        self._set_behavior('stuck_turn')
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
                'unstick': ({'phase': self._unstick['phase'], 'count': len(self._unstick_times)}
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
