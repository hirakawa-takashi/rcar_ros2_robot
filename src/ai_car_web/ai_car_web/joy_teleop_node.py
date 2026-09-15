#!/usr/bin/env python3
"""Logitech F710 ゲームパッドによる手動操作ノード。

Linux joystick API (/dev/input/js*) を直接読み、正規化済み (-1.0〜1.0) の
Twist を /joy_cmd へ publish する。実際の速度スケール・障害物ガード・
タイムアウト停止は dashboard_node が /cmd_vel へ変換する際に行うため、
Web 画面からの操作と同じ安全機構が適用される。

F710 は背面スイッチを X（XInput）側にすること。xpad ドライバでの割り当て:
  軸  0: 左スティック X   1: 左スティック Y   2: LT
      3: 右スティック X   4: 右スティック Y   5: RT
      6: 十字キー X       7: 十字キー Y
  ボタン 0:A 1:B 2:X 3:Y 4:LB 5:RB 6:BACK 7:START 8:MODE 9:左スティック押 10:右スティック押
"""

import array
import fcntl
import os
import select
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

JS_EVENT_FMT = 'IhBB'  # time(ms), value, type, number
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
JSIOCGNAME = 0x80806a13
AXIS_MAX = 32767.0


class JoyTeleopNode(Node):
    """ゲームパッドの入力を正規化した Twist として publish する。"""

    def __init__(self):
        super().__init__('joy_teleop_node')
        self.declare_parameter('device', '/dev/input/js0')
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('deadzone', 0.15)
        # 通常時の出力割合。RB を押している間は 1.0（全速）
        self.declare_parameter('speed_scale', 0.5)
        self.declare_parameter('axis_linear_x', 1)
        self.declare_parameter('axis_linear_y', 0)
        self.declare_parameter('axis_angular_z', 3)
        self.declare_parameter('axis_dpad_x', 6)
        self.declare_parameter('axis_dpad_y', 7)
        self.declare_parameter('button_turbo', 5)
        self.declare_parameter('button_stop', 1)
        # -1 で無効。指定時はそのボタンを押している間だけ動く（デッドマン）
        self.declare_parameter('button_enable', -1)

        self.device = self.get_parameter('device').value
        self.deadzone = float(self.get_parameter('deadzone').value)
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.axis_lx = int(self.get_parameter('axis_linear_x').value)
        self.axis_ly = int(self.get_parameter('axis_linear_y').value)
        self.axis_az = int(self.get_parameter('axis_angular_z').value)
        self.axis_dx = int(self.get_parameter('axis_dpad_x').value)
        self.axis_dy = int(self.get_parameter('axis_dpad_y').value)
        self.btn_turbo = int(self.get_parameter('button_turbo').value)
        self.btn_stop = int(self.get_parameter('button_stop').value)
        self.btn_enable = int(self.get_parameter('button_enable').value)

        self.pub = self.create_publisher(
            Twist, self.get_parameter('joy_cmd_topic').value, 10)

        self._lock = threading.Lock()
        self._axes = {}
        self._buttons = {}
        self._connected = False
        self._stop = threading.Event()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._publish_cb)

    # --- デバイス読み取り ---
    def _read_loop(self):
        """デバイスを開いてイベントを読み続ける。抜かれたら再接続を待つ。"""
        while not self._stop.is_set():
            try:
                fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            except OSError:
                self._set_connected(False)
                time.sleep(1.0)
                continue
            try:
                name = array.array('B', [0] * 128)
                fcntl.ioctl(fd, JSIOCGNAME, name)
                label = name.tobytes().split(b'\0')[0].decode(errors='replace')
                self.get_logger().info(f'ゲームパッド接続: {label} ({self.device})')
                self._set_connected(True)
                self._pump(fd)
            except OSError as e:
                self.get_logger().warn(f'ゲームパッドが切断されました: {e}')
            finally:
                os.close(fd)
                self._set_connected(False)
                time.sleep(1.0)

    def _pump(self, fd):
        while not self._stop.is_set():
            r, _, _ = select.select([fd], [], [], 0.5)
            if not r:
                continue
            data = os.read(fd, JS_EVENT_SIZE * 64)
            if not data:
                raise OSError('EOF')
            for off in range(0, len(data) - JS_EVENT_SIZE + 1, JS_EVENT_SIZE):
                _, value, ev_type, number = struct.unpack_from(JS_EVENT_FMT, data, off)
                ev_type &= ~JS_EVENT_INIT
                with self._lock:
                    if ev_type == JS_EVENT_AXIS:
                        self._axes[number] = value / AXIS_MAX
                    elif ev_type == JS_EVENT_BUTTON:
                        self._buttons[number] = bool(value)

    def _set_connected(self, connected: bool):
        with self._lock:
            if self._connected and not connected:
                self._axes.clear()
                self._buttons.clear()
            self._connected = connected

    # --- 指令生成 ---
    def _axis(self, idx: int) -> float:
        v = self._axes.get(idx, 0.0)
        if abs(v) < self.deadzone:
            return 0.0
        # デッドゾーン外を 0〜1 に再スケール
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - self.deadzone) / (1.0 - self.deadzone)

    def _publish_cb(self):
        with self._lock:
            if not self._connected:
                return
            if self.btn_stop >= 0 and self._buttons.get(self.btn_stop, False):
                self.pub.publish(Twist())
                return
            if self.btn_enable >= 0 and not self._buttons.get(self.btn_enable, False):
                self.pub.publish(Twist())
                return
            # スティック上 / 左が負値なので符号を反転して ROS 座標系（前 +x, 左 +y, 左旋回 +z）に合わせる
            lx = -self._axis(self.axis_lx)
            ly = -self._axis(self.axis_ly)
            az = -self._axis(self.axis_az)
            # 十字キーはデジタル入力としてスティックの代わりに使える
            dx = self._axes.get(self.axis_dx, 0.0)
            dy = self._axes.get(self.axis_dy, 0.0)
            if abs(dy) > 0.5:
                lx = -1.0 if dy > 0 else 1.0
            if abs(dx) > 0.5:
                ly = -1.0 if dx > 0 else 1.0
            scale = 1.0 if self._buttons.get(self.btn_turbo, False) else self.speed_scale
        msg = Twist()
        msg.linear.x = lx * scale
        msg.linear.y = ly * scale
        msg.angular.z = az * scale
        self.pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleopNode()
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
