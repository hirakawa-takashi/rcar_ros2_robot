#!/usr/bin/env python3
"""運転モード管理ノード（手動 / 自動 / 停止）。

/cmd_vel を publish するのはこのノードだけにし、モードに応じて
  MANUAL: /cmd_vel_manual（dashboard_node = Web 操作 + ゲームパッド）
  AUTO  : /cmd_vel_auto（autonomy_node）
  STOP  : ゼロ速度
の一方だけを通すマルチプレクサ。モードの切替要求は /drive_mode_request
（std_msgs/String: "manual" / "auto" / "stop"）で受け、現在の状態を
/drive_mode（JSON）で配信する。

安全ルール:
  - 手動優先: AUTO 中に手動指令（スティック / Web）が来たら MANUAL へ自動復帰
  - AUTO 進入条件（LiDAR 生存・障害物停止でない・手動ニュートラル・自動指令生存・
    ゲームパッド接続）を満たさないと拒否し、理由を /drive_mode に載せる
  - /cmd_vel_auto が途絶えたら STOP、ゲームパッド切断時も STOP（パラメータで選択）
  - モード遷移時は必ず一度ゼロ速度を送る
  - 障害物 "stop" 判定中はどのモードでも前進を止める（最終段ガード）
"""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

MODES = ('manual', 'auto', 'stop')
LABELS = {'manual': '手動', 'auto': '自動', 'stop': '停止'}


def _moving(msg: Twist, eps: float = 1e-3) -> bool:
    return (abs(msg.linear.x) > eps or abs(msg.linear.y) > eps
            or abs(msg.angular.z) > eps)


class DriveModeNode(Node):
    """モードに応じて手動 / 自動の速度指令を /cmd_vel へ中継する。"""

    def __init__(self):
        super().__init__('drive_mode_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('manual_topic', '/cmd_vel_manual')
        self.declare_parameter('auto_topic', '/cmd_vel_auto')
        self.declare_parameter('request_topic', '/drive_mode_request')
        self.declare_parameter('mode_topic', '/drive_mode')
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('initial_mode', 'manual')
        self.declare_parameter('publish_rate', 5.0)
        # 自動指令がこの秒数途絶えたら STOP
        self.declare_parameter('auto_timeout', 0.5)
        # 障害物判定がこの秒数より古ければ LiDAR 停止とみなす
        self.declare_parameter('obstacle_timeout', 2.0)
        # ゲームパッド指令（20Hz）がこの秒数途絶えたら未接続
        self.declare_parameter('joy_timeout', 1.0)
        # AUTO 進入・継続にゲームパッド接続を必須にする（切断で STOP）
        self.declare_parameter('auto_requires_joy', True)
        # AUTO 中に手動指令が来たら MANUAL へ戻す（手動優先）
        self.declare_parameter('manual_override', True)
        # 最終段の障害物ガード（"stop" 判定で前進を止める）
        self.declare_parameter('obstacle_guard', True)

        self.auto_timeout = float(self.get_parameter('auto_timeout').value)
        self.obstacle_timeout = float(self.get_parameter('obstacle_timeout').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        self.auto_requires_joy = bool(self.get_parameter('auto_requires_joy').value)
        self.manual_override = bool(self.get_parameter('manual_override').value)
        self.obstacle_guard = bool(self.get_parameter('obstacle_guard').value)

        self._lock = threading.Lock()
        mode = str(self.get_parameter('initial_mode').value).lower()
        self._mode = mode if mode in MODES else 'manual'
        self._since = time.monotonic()
        self._source = 'init'
        self._reason = '起動時の既定モード'
        self._last_reject = None
        self._manual_stamp = 0.0
        self._manual_moving = False
        self._auto_stamp = 0.0
        self._auto_cmd = Twist()
        self._joy_stamp = 0.0
        self._joy_moving = False
        self._obstacle = None
        self._obstacle_stamp = 0.0
        self._last_out = Twist()
        self._last_out_time = 0.0

        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.mode_pub = self.create_publisher(
            String, self.get_parameter('mode_topic').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            Twist, self.get_parameter('manual_topic').value, self._manual_cb, 10)
        self.create_subscription(
            Twist, self.get_parameter('auto_topic').value, self._auto_cb, 10)
        self.create_subscription(
            String, self.get_parameter('request_topic').value, self._request_cb, 10)
        self.create_subscription(
            String, self.get_parameter('obstacle_topic').value, self._obstacle_cb, 10)
        joy_topic = self.get_parameter('joy_cmd_topic').value
        if joy_topic:
            self.create_subscription(Twist, joy_topic, self._joy_cb, 10)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        self.create_timer(0.05, self._auto_relay_cb)
        self._publish_mode()
        self.get_logger().info(f'運転モード: {LABELS[self._mode]}')

    # --- 入力 ---
    def _manual_cb(self, msg: Twist):
        moving = _moving(msg)
        with self._lock:
            self._manual_stamp = time.monotonic()
            self._manual_moving = moving
            mode = self._mode
        if mode == 'manual':
            self._publish_cmd(msg)
        elif mode == 'auto' and moving and self.manual_override:
            self._set_mode('manual', 'override', '手動指令を検出したため手動へ復帰')
            self._publish_cmd(msg)
        # STOP 中の手動指令は無視（明示的に MANUAL へ戻す必要がある）

    def _auto_cb(self, msg: Twist):
        with self._lock:
            self._auto_stamp = time.monotonic()
            self._auto_cmd = msg

    def _auto_relay_cb(self):
        """AUTO 中は自動指令を 20Hz で中継し、途絶えたら STOP へ落とす。"""
        with self._lock:
            if self._mode != 'auto':
                return
            age = time.monotonic() - self._auto_stamp
            cmd = self._auto_cmd
        if age > self.auto_timeout:
            self._set_mode('stop', 'watchdog', '自動指令が途絶えたため停止')
            return
        block = self._auto_block_reasons(continuing=True)
        if block:
            self._set_mode('stop', 'guard', ' / '.join(block))
            return
        self._publish_cmd(cmd)

    def _joy_cb(self, msg: Twist):
        with self._lock:
            self._joy_stamp = time.monotonic()
            self._joy_moving = _moving(msg)

    def _obstacle_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            with self._lock:
                self._obstacle = payload
                self._obstacle_stamp = time.monotonic()

    def _request_cb(self, msg: String):
        text = msg.data.strip()
        source = 'request'
        try:
            payload = json.loads(text)
            if isinstance(payload, dict):
                source = str(payload.get('source') or source)
                text = str(payload.get('mode') or '')
        except (TypeError, json.JSONDecodeError):
            pass
        self.request(text.lower(), source)

    # --- モード管理 ---
    def request(self, mode: str, source: str):
        """モード切替要求。AUTO は進入条件を満たさないと拒否する。"""
        if mode == 'toggle':
            mode = 'manual' if self._mode == 'auto' else 'auto'
        if mode not in MODES:
            self.get_logger().warn(f'不明なモード要求: {mode!r} ({source})')
            return False
        if mode == 'auto':
            block = self._auto_block_reasons(entering=True)
            if block:
                reason = ' / '.join(block)
                with self._lock:
                    self._last_reject = {
                        'mode': mode, 'source': source, 'reason': reason,
                        'stamp': time.time()}
                self.get_logger().warn(f'自動運転への切替を拒否 ({source}): {reason}')
                self._publish_mode()
                return False
        self._set_mode(mode, source, f'{source} からの切替要求')
        return True

    def _auto_block_reasons(self, entering: bool = False, continuing: bool = False):
        """AUTO を許可できない理由の一覧を返す（空なら許可）。

        continuing=True は AUTO 継続判定で、障害物 "stop" は理由に含めない
        （最終段ガードで前進だけ止め、回避行動は autonomy_node に任せる）。
        """
        now = time.monotonic()
        reasons = []
        with self._lock:
            obstacle = self._obstacle
            obs_age = now - self._obstacle_stamp if self._obstacle_stamp else None
            joy_age = now - self._joy_stamp if self._joy_stamp else None
            joy_moving = self._joy_moving
            manual_moving = self._manual_moving and now - self._manual_stamp < 0.5
            auto_age = now - self._auto_stamp if self._auto_stamp else None
        if obstacle is None or obs_age is None or obs_age > self.obstacle_timeout:
            reasons.append('障害物判定（LiDAR）が受信できていません')
        else:
            level = obstacle.get('level')
            if level == 'stop' and not continuing:
                reasons.append(f"前方に障害物（{obstacle.get('reason') or 'stop'}）")
            elif level == 'unknown':
                reasons.append('LiDAR データなし')
        if self.auto_requires_joy and (joy_age is None or joy_age > self.joy_timeout):
            reasons.append('ゲームパッドが未接続です')
        if entering:
            if joy_moving or manual_moving:
                reasons.append('スティック / 手動操作をニュートラルに戻してください')
            if auto_age is None or auto_age > max(self.auto_timeout, 1.0):
                reasons.append('自律走行ノード（/cmd_vel_auto）が動いていません')
        return reasons

    def _set_mode(self, mode: str, source: str, reason: str):
        with self._lock:
            if mode == self._mode:
                return
            prev = self._mode
            self._mode = mode
            self._since = time.monotonic()
            self._source = source
            self._reason = reason
            self._last_reject = None
        # 遷移時は必ず一度ゼロ速度を送る（急変防止）
        self._publish_cmd(Twist())
        self.get_logger().info(
            f'運転モード {LABELS[prev]} → {LABELS[mode]} ({source}: {reason})')
        self._publish_mode()

    # --- 出力 ---
    def _publish_cmd(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x
        out.linear.y = msg.linear.y
        out.angular.z = msg.angular.z
        if self.obstacle_guard and out.linear.x > 0.0 and self._obstacle_stop():
            out.linear.x = 0.0
        with self._lock:
            self._last_out = out
            self._last_out_time = time.monotonic()
        self.cmd_pub.publish(out)

    def _obstacle_stop(self) -> bool:
        with self._lock:
            obstacle = self._obstacle
            age = time.monotonic() - self._obstacle_stamp if self._obstacle_stamp else None
        if not obstacle or age is None or age > self.obstacle_timeout:
            return False
        return obstacle.get('level') == 'stop'

    def state(self):
        now = time.monotonic()
        with self._lock:
            mode = self._mode
            joy_connected = bool(self._joy_stamp) and now - self._joy_stamp < self.joy_timeout
            auto_alive = bool(self._auto_stamp) and now - self._auto_stamp < max(
                self.auto_timeout, 1.0)
            manual_alive = bool(self._manual_stamp) and now - self._manual_stamp < 2.0
            payload = {
                'stamp': round(time.time(), 3),
                'mode': mode,
                'label': LABELS[mode],
                'since_s': round(now - self._since, 1),
                'source': self._source,
                'reason': self._reason,
                'last_reject': self._last_reject,
                'inputs': {
                    'joy_connected': joy_connected,
                    'auto_alive': auto_alive,
                    'manual_alive': manual_alive,
                },
                'cmd_vel': {
                    'linear_x': round(self._last_out.linear.x, 3),
                    'linear_y': round(self._last_out.linear.y, 3),
                    'angular_z': round(self._last_out.angular.z, 3),
                },
            }
        block = self._auto_block_reasons(entering=(mode != 'auto'))
        payload['auto_ok'] = not block
        payload['auto_block_reasons'] = block
        return payload

    def _publish_mode(self):
        msg = String()
        msg.data = json.dumps(self.state(), ensure_ascii=False)
        self.mode_pub.publish(msg)

    def _timer_cb(self):
        with self._lock:
            mode = self._mode
        if mode == 'stop':
            # STOP 中はゼロ速度を定期的に送り続ける（下流のタイムアウトに依存しない）
            self._publish_cmd(Twist())
        self._publish_mode()


def main(args=None):
    rclpy.init(args=args)
    node = DriveModeNode()
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
