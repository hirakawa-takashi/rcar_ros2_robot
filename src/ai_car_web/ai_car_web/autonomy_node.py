#!/usr/bin/env python3
"""自律走行ノード（Level 1: LiDAR 反応型）。

地図や自己位置を使わず、/scan と /obstacle_status だけで
  前進 → 接近したら減速 → 停止距離内なら空いている側へ旋回 → 前進
を繰り返す。速度指令は /cmd_vel_auto に常時（20Hz）publish し、
drive_mode_node が AUTO モードのときだけ /cmd_vel へ中継する。
AUTO 以外のモードでは状態を "standby" にしてゼロ速度を出し続ける
（drive_mode_node の生存確認に使われる）。

行動（/autonomy_status の behavior）:
  standby   AUTO 待機（ゼロ速度）
  forward   前進
  slow      減速前進
  turn      その場旋回で空いている方向を探す
  backoff   前後左右とも近いときに少し後退
  cliff_back  前の落下防止センサーが段差 → /odom で cliff_backoff_m 下がる
  cliff_turn  段差から下がったあと、空いている側へ cliff_turn_deg 回って走行を続ける
  cliff_stuck 前後とも段差・/odom なしなどで動けない（ゼロ速度）
  blocked   LiDAR 未受信などで停止
"""

import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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
}


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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
        # 段差（/cliff_status）を見たら、/odom で下がって回り、走行を続ける
        self.declare_parameter('cliff_topic', '/cliff_status')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('cliff_timeout', 1.0)
        self.declare_parameter('odom_timeout', 0.5)
        self.declare_parameter('cliff_backoff_m', 0.10)
        self.declare_parameter('cliff_turn_deg', 120.0)
        self.declare_parameter('cliff_escape_timeout', 20.0)

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
        self.cliff_timeout = float(g('cliff_timeout').value)
        self.odom_timeout = float(g('odom_timeout').value)
        self.cliff_backoff_m = float(g('cliff_backoff_m').value)
        self.cliff_turn = math.radians(float(g('cliff_turn_deg').value))
        self.cliff_escape_timeout = float(g('cliff_escape_timeout').value)

        self._lock = threading.Lock()
        self._mode = None
        self._sectors = None   # {'front','left','right','back'} 最短距離
        self._scan_stamp = 0.0
        self._obstacle = None
        self._obstacle_stamp = 0.0
        self._behavior = 'standby'
        self._behavior_since = time.monotonic()
        self._turn_dir = 1.0
        self._last_cmd = Twist()
        self._cliff = None
        self._cliff_stamp = 0.0
        self._pose = None      # (x, y, yaw)
        self._pose_stamp = 0.0
        self._escape = None    # 段差から離れる途中の状態

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
        rate = float(g('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        self.create_timer(0.2, self._status_cb)

    # --- 入力 ---
    def _scan_cb(self, msg: LaserScan):
        sectors = {'front': [], 'left': [], 'right': [], 'back': []}
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment + self.offset
            a = math.atan2(math.sin(a), math.cos(a))
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
        if mode != 'auto':
            self._escape = None
            self._set_behavior('standby')
            return cmd
        if s is None or scan_age is None or scan_age > self.scan_max_age:
            self._set_behavior('blocked')
            return cmd

        cliff_front = bool(cliff and cliff.get('block_forward'))
        cliff_back = bool(cliff and cliff.get('block_backward'))
        if cliff_front and (self._escape is None or self._escape['phase'] == 'turn'):
            self._escape = {'phase': 'back', 'since': now, 'start': pose}
            self.get_logger().info('前に段差: 下がって回る')
        if self._escape is not None:
            esc_cmd = self._escape_step(now, s, pose, cliff_front, cliff_back)
            if esc_cmd is not None:
                return esc_cmd

        front = s.get('front')
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

        if in_turn and (front_v < self.clear_d or not turning_long_enough):
            cmd.angular.z = self._turn_dir * self.turn
            return cmd

        if front_v <= self.stop_d:
            # 空いている側へ旋回（左が広ければ左旋回 = +z）
            fl = s.get('front_left')
            fr = s.get('front_right')
            left_score = min(left_v, fl if fl is not None else big)
            right_score = min(right_v, fr if fr is not None else big)
            self._turn_dir = 1.0 if left_score >= right_score else -1.0
            self._set_behavior('turn')
            cmd.angular.z = self._turn_dir * self.turn
            return cmd

        if front_v <= self.slow_d:
            self._set_behavior('slow')
            cmd.linear.x = self.slow
            # 近い側から少し離れる方向へ緩く曲がる
            fl = s.get('front_left')
            fr = s.get('front_right')
            if fl is not None and fr is not None and abs(fl - fr) > 0.1:
                cmd.angular.z = (0.3 * self.turn) * (1.0 if fl > fr else -1.0)
            return cmd

        self._set_behavior('forward')
        cmd.linear.x = self.cruise
        return cmd

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
        with self._lock:
            self._last_cmd = cmd
        self.cmd_pub.publish(cmd)

    def _status_cb(self):
        now = time.monotonic()
        with self._lock:
            payload = {
                'stamp': round(time.time(), 3),
                'level': 1,
                'method': 'LiDAR 反応型（地図なし）',
                'mode': self._mode,
                'behavior': self._behavior,
                'behavior_label': LABELS.get(self._behavior, self._behavior),
                'behavior_s': round(now - self._behavior_since, 1),
                'turn_dir': 'left' if self._turn_dir > 0 else 'right',
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
