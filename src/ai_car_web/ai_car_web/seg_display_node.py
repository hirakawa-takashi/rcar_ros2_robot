#!/usr/bin/env python3
"""TM1637 4桁7セグメントLEDへロボット状態を表示するノード。"""

import json
import time

try:
    import gpiod
except ImportError:
    gpiod = None

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

SEGMENTS = {
    '0': 0x3F, '1': 0x06, '2': 0x5B, '3': 0x4F, '4': 0x66,
    '5': 0x6D, '6': 0x7D, '7': 0x07, '8': 0x7F, '9': 0x6F,
    'A': 0x77, 'b': 0x7C, 'C': 0x39, 'd': 0x5E, 'E': 0x79,
    'F': 0x71, 'H': 0x76, 'J': 0x1E, 'L': 0x38, 'n': 0x54,
    'o': 0x5C, 'O': 0x3F, 'P': 0x73, 'r': 0x50, 'S': 0x6D,
    't': 0x78, 'U': 0x3E, 'y': 0x6E, '-': 0x40, ' ': 0x00,
}


class TM1637Display:
    """libgpiod v1 API で TM1637 を bit-bang 駆動する。"""

    def __init__(self, chip, clk_gpio: int, dio_gpio: int):
        self.chip = chip
        self.clk = chip.get_line(clk_gpio)
        self.dio = chip.get_line(dio_gpio)
        try:
            self.clk.request(
                consumer='seg_display_node', type=gpiod.LINE_REQ_DIR_OUT)
            self.dio.request(
                consumer='seg_display_node', type=gpiod.LINE_REQ_DIR_OUT)
            self.clk.set_value(1)
            self.dio.set_value(1)
        except Exception:
            self.close()
            raise
        self._last_text = None
        self._last_brightness = None
        self._last_send = 0.0

    @staticmethod
    def _delay():
        time.sleep(2e-6)

    def _clock(self, value: int):
        self.clk.set_value(value)
        self._delay()

    def _start(self):
        self.clk.set_value(1)
        self.dio.set_value(1)
        self._delay()
        self.dio.set_value(0)
        self._delay()
        self.clk.set_value(0)
        self._delay()

    def _stop(self):
        self.clk.set_value(0)
        self.dio.set_value(0)
        self._delay()
        self._clock(1)
        self.dio.set_value(1)
        self._delay()

    def _write_byte(self, value: int):
        for bit in range(8):
            self.dio.set_value((value >> bit) & 1)
            self._delay()
            self._clock(1)
            self._clock(0)
        # ACK は読まず、DIO を High にして CLK をパルスする。
        self.dio.set_value(1)
        self._delay()
        self._clock(1)
        self._clock(0)

    def _write_command(self, values):
        self._start()
        for value in values:
            self._write_byte(value)
        self._stop()

    def show(self, text: str, brightness: int):
        text = (text[:4]).ljust(4)
        brightness = max(0, min(int(brightness), 7))
        now = time.monotonic()
        if (text == self._last_text
                and brightness == self._last_brightness
                and now - self._last_send < 2.0):
            return
        segments = [SEGMENTS.get(char, 0x00) for char in text]
        self._write_command([0x40])
        self._write_command([0xC0, *segments])
        self._write_command([0x88 | brightness])
        self._last_text = text
        self._last_brightness = brightness
        self._last_send = now

    def close(self):
        for line in (getattr(self, 'clk', None), getattr(self, 'dio', None)):
            if line is not None:
                try:
                    line.release()
                except Exception:
                    pass
        chip = getattr(self, 'chip', None)
        if chip is not None and hasattr(chip, 'close'):
            try:
                chip.close()
            except Exception:
                pass
        self.clk = None
        self.dio = None
        self.chip = None


class SegDisplayNode(Node):
    """各種トピックからロボット状態を判定して7セグメントLEDへ表示する。"""

    def __init__(self):
        super().__init__('seg_display_node')
        self.declare_parameter('gpio_chip', '')
        self.declare_parameter('clk_gpio', 23)
        self.declare_parameter('dio_gpio', 24)
        self.declare_parameter('brightness', 2)
        self.declare_parameter('update_rate', 5.0)
        self.declare_parameter('system_status_topic', '/system_status')
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('state_topic', '/display_state')
        self.declare_parameter('status_timeout', 3.0)

        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.clk_gpio = int(self.get_parameter('clk_gpio').value)
        self.dio_gpio = int(self.get_parameter('dio_gpio').value)
        self.brightness = int(self.get_parameter('brightness').value)
        self.status_timeout = float(self.get_parameter('status_timeout').value)
        self._display = None
        self._next_display_retry = 0.0
        self._last_display_error = 0.0
        self._missing_gpiod_logged = False
        self._start_time = time.monotonic()
        self._system = None
        self._system_stamp = None
        self._obstacle = None
        self._obstacle_stamp = None
        self._last_motion = None
        self._state = None

        self._state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(
            String, self.get_parameter('system_status_topic').value,
            self._system_cb, 10)
        self.create_subscription(
            String, self.get_parameter('obstacle_topic').value,
            self._obstacle_cb, 10)
        self.create_subscription(
            Twist, self.get_parameter('joy_cmd_topic').value,
            self._motion_cb, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._motion_cb, 10)
        rate = float(self.get_parameter('update_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)

    def _system_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._system = payload
            self._system_stamp = time.monotonic()

    def _obstacle_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(payload, dict):
            self._obstacle = payload
            self._obstacle_stamp = time.monotonic()

    def _motion_cb(self, msg: Twist):
        if (abs(msg.linear.x) > 0.05 or abs(msg.linear.y) > 0.05
                or abs(msg.angular.z) > 0.05):
            self._last_motion = time.monotonic()

    def _warn_display_error(self, message: str, error):
        now = time.monotonic()
        if now - self._last_display_error >= 5.0:
            self.get_logger().warn(f'{message}: {error}')
            self._last_display_error = now

    def _find_chip(self):
        if self.gpio_chip:
            return gpiod.Chip(self.gpio_chip)
        for index in range(10):
            path = f'/dev/gpiochip{index}'
            try:
                chip = gpiod.Chip(path)
            except OSError:
                continue
            if chip.label() == 'pinctrl-rp1':
                return chip
            if hasattr(chip, 'close'):
                chip.close()
        raise OSError('pinctrl-rp1 の gpiochip が見つかりません')

    def _ensure_display(self):
        if gpiod is None:
            if not self._missing_gpiod_logged:
                self.get_logger().warn(
                    'python3-libgpiod がないため7セグ表示を無効化します')
                self._missing_gpiod_logged = True
            return False
        if self._display is not None:
            return True
        now = time.monotonic()
        if now < self._next_display_retry:
            return False
        try:
            chip = self._find_chip()
            self._display = TM1637Display(
                chip, self.clk_gpio, self.dio_gpio)
            self.get_logger().info('TM1637 7セグ表示を接続しました')
        except Exception as e:
            self._display = None
            self._warn_display_error('TM1637 7セグ表示に接続できません', e)
            self._next_display_retry = now + 2.0
            return False
        return True

    def _close_display(self):
        if self._display is not None:
            self._display.close()
            self._display = None

    def _state_code(self, now: float) -> str:
        system_missing = (
            self._system_stamp is None
            or now - self._system_stamp >= self.status_timeout)
        if system_missing:
            if (self._system_stamp is None
                    and now - self._start_time < 30.0):
                return 'boot'
            return 'Err '

        power = self._system.get('power') or {}
        throttled = power.get('throttled') or {}
        if throttled.get('under_voltage_now', False):
            return 'LoU '

        obstacle_fresh = (
            self._obstacle_stamp is not None
            and now - self._obstacle_stamp < self.status_timeout)
        if obstacle_fresh:
            level = self._obstacle.get('level')
            if level == 'stop':
                return 'ObSt'
            if level == 'slow':
                return 'SLo '

        if self._last_motion is not None and now - self._last_motion < 1.0:
            return 'HAnd'
        return 'rdy '

    def _timer_cb(self):
        now = time.monotonic()
        code = self._state_code(now)
        if code != self._state:
            self._state = code
            msg = String()
            msg.data = code
            self._state_pub.publish(msg)
            self.get_logger().info(f'表示状態: {code}')

        if not self._ensure_display():
            return
        try:
            self._display.show(code, self.brightness)
        except Exception as e:
            self._warn_display_error('TM1637 表示エラー', e)
            self._close_display()
            self._next_display_retry = time.monotonic() + 2.0

    def destroy_node(self):
        if self._display is not None:
            try:
                self._display.show('    ', self.brightness)
            except Exception:
                pass
        self._close_display()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SegDisplayNode()
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
