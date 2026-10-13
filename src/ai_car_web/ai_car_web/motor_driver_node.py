#!/usr/bin/env python3
"""/cmd_vel を Adafruit Motor HAT（PCA9685 + TB6612 ×2）の M1〜M4 に出すノード。

メカナムホイールの逆運動学で 4 輪の出力（-1〜1）を出し、PWM のデューティにする
（エンコーダーを使わないオープンループ）。端子と車輪の割り付け・向き（reversed）は
motor_hat.yaml。/cmd_vel が timeout 秒届かないと全モーターを止める。
旋回の指令が 0 で前後・左右に動くあいだは、IMU（/imu/data）の向きを動き始めの向きに
保つように旋回を足す（heading_hold）。IMU が届かないときは足さない。
I2C は /dev/i2c-N を直接使う（smbus2 などの追加ライブラリなし）。
"""

import fcntl
import json
import math
import os
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from ai_car_web.motor_hat import load_motor_hat

I2C_SLAVE = 0x0703
MODE1 = 0x00
MODE2 = 0x01
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD
PRESCALE = 0xFE
FULL = 0x1000
OSC_HZ = 25_000_000


class PCA9685:
    """/dev/i2c-N 経由で PCA9685 の 16 チャンネルを設定する。"""

    def __init__(self, bus: int, address: int, frequency: float):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
            self._write(ALL_LED_OFF_H, 0x10)
            prescale = max(3, min(255, round(OSC_HZ / 4096 / frequency) - 1))
            self._write(MODE1, 0x10)
            self._write(PRESCALE, prescale)
            self._write(MODE1, 0x00)
            time.sleep(0.005)
            self._write(MODE2, 0x04)
            self._write(MODE1, 0xA0)
            time.sleep(0.005)
            self._write(ALL_LED_OFF_H, 0x00)
        except OSError:
            self.close()
            raise

    def close(self):
        fd, self.fd = self.fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _write(self, reg: int, *data: int):
        payload = bytes([reg, *data])
        if os.write(self.fd, payload) != len(payload):
            raise OSError(f'I2C 書き込み失敗: 0x{reg:02x}')

    def set(self, channel: int, on: int, off: int):
        self._write(LED0_ON_L + 4 * channel,
                    on & 0xFF, on >> 8, off & 0xFF, off >> 8)

    def all_off(self):
        self._write(ALL_LED_OFF_H, 0x10)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def heading_correction(target: float, yaw: float, yaw_rate: float,
                       kp: float, kd: float, limit: float, deadband: float = 0.0) -> float:
    """向き target [rad] にもどす旋回 [rad/s]（左回り +）を返す。ずれが deadband [rad] 以下は 0。"""
    err = _wrap(target - yaw)
    err = math.copysign(max(0.0, abs(err) - deadband), err)
    w = kp * err - kd * yaw_rate
    return max(-limit, min(limit, w))


def mecanum(vx: float, vy: float, wz: float) -> dict:
    """前 +・左 +・左回り + の正規化指令から 4 輪の出力（-1〜1）を返す。"""
    out = {
        'front_left': vx - vy - wz,
        'front_right': vx + vy + wz,
        'rear_left': vx + vy - wz,
        'rear_right': vx - vy + wz,
    }
    peak = max(1.0, *(abs(v) for v in out.values()))
    return {k: v / peak for k, v in out.items()}


class MotorDriverNode(Node):
    """/cmd_vel を受けて Motor HAT の 4 モーターを回す。"""

    def __init__(self):
        super().__init__('motor_driver_node')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('status_topic', '/motor_status')
        self.declare_parameter('motor_hat_config', '')
        # dashboard_node の max_linear_speed / max_angular_speed と同じ値で出力 100 %
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        # 出力 100 % のときのデューティ（0〜1）
        self.declare_parameter('max_duty', 1.0)
        # /cmd_vel がこの秒数届かないと止める
        self.declare_parameter('timeout', 0.5)
        # 旋回の指令が 0 で動くあいだ、IMU の向きを保つ
        self.declare_parameter('heading_hold', True)
        self.declare_parameter('imu_topic', '/imu/data')
        # 向きのずれ 1 rad あたりの旋回 [rad/s]・回る速さを打ち消す強さ・足す旋回の上限 [rad/s]
        # 実行中に ros2 param set で変えられる
        self.declare_parameter('heading_kp', 0.25)
        self.declare_parameter('heading_kd', 0.05)
        self.declare_parameter('heading_max_correction', 0.05)
        # このずれ [deg] までは直さない（IMU のゆれで左右に振らないように）
        self.declare_parameter('heading_deadband_deg', 1.0)
        # IMU がこの秒数届かないと向きを保たない
        self.declare_parameter('imu_timeout', 0.5)

        path = self.get_parameter('motor_hat_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'motor_hat.yaml')
        cfg = load_motor_hat(path)
        if cfg['error']:
            self.get_logger().error(f'Motor HAT の割り付け: {cfg["error"]}')
        for w in cfg['warnings']:
            self.get_logger().warn(f'Motor HAT の割り付け: {w}')
        hat = cfg['hat']
        self.bus = int(hat.get('i2c_bus', 1))
        addr = hat.get('i2c_address', '0x60')
        self.address = int(addr, 16) if isinstance(addr, str) else int(addr)
        self.frequency = float(hat.get('pwm_frequency', 1600))
        self.motors = [m for m in cfg['motors'] if m['wheel']]
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.max_duty = max(0.0, min(1.0, float(self.get_parameter('max_duty').value)))
        self.timeout = float(self.get_parameter('timeout').value)
        self.heading_hold = bool(self.get_parameter('heading_hold').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.heading_kd = float(self.get_parameter('heading_kd').value)
        self.heading_limit = float(self.get_parameter('heading_max_correction').value)
        self.heading_deadband = math.radians(float(self.get_parameter('heading_deadband_deg').value))
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)

        self.pca = None
        self.error = ''
        self._next_open = 0.0
        self._twist = (0.0, 0.0, 0.0)
        self._cmd_time = 0.0
        self._yaw = None
        self._yaw_rate = 0.0
        self._imu_time = 0.0
        self._hold_target = None
        self._correction = 0.0
        self._written = {}
        self._connect()

        self.status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value, self._cmd_cb, 10)
        if self.heading_hold:
            self.create_subscription(
                Imu, self.get_parameter('imu_topic').value, self._imu_cb, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.create_timer(0.05, self._update)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            f'Motor HAT i2c-{self.bus} 0x{self.address:02X}: '
            + ', '.join(f'{m["label"]}{"（逆）" if m["reversed"] else ""}' for m in self.motors))

    def _on_set_parameters(self, params):
        """heading_hold の強さを実行中に変えられるようにする。"""
        attrs = {
            'heading_kp': lambda v: setattr(self, 'heading_kp', float(v)),
            'heading_kd': lambda v: setattr(self, 'heading_kd', float(v)),
            'heading_max_correction': lambda v: setattr(self, 'heading_limit', float(v)),
            'heading_deadband_deg': lambda v: setattr(self, 'heading_deadband', math.radians(float(v))),
        }
        for p in params:
            if p.name in attrs:
                attrs[p.name](p.value)
        return SetParametersResult(successful=True)

    def _connect(self):
        now = time.monotonic()
        if self.pca or now < self._next_open:
            return
        try:
            self.pca = PCA9685(self.bus, self.address, self.frequency)
            self._written = {}
            self.error = ''
            self.get_logger().info('Motor HAT に接続しました')
        except OSError as e:
            self.pca = None
            self._next_open = now + 2.0
            if self.error != str(e):
                self.get_logger().warn(f'Motor HAT に接続できません: {e}')
            self.error = str(e)

    def _cmd_cb(self, msg: Twist):
        self._twist = (msg.linear.x, msg.linear.y, msg.angular.z)
        self._cmd_time = time.monotonic()

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                               1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._yaw_rate = msg.angular_velocity.z
        self._imu_time = time.monotonic()

    def _update_heading(self, vx: float, vy: float, wz: float) -> float:
        """向きを保つために足す旋回 [rad/s] を返す。保たないときは目標の向きを捨てる。"""
        moving = abs(vx) > 1e-3 or abs(vy) > 1e-3
        imu_ok = (self._yaw is not None
                  and time.monotonic() - self._imu_time <= self.imu_timeout)
        if not (self.heading_hold and moving and abs(wz) <= 1e-3 and imu_ok):
            self._hold_target = None
            return 0.0
        if self._hold_target is None:
            self._hold_target = self._yaw
        return heading_correction(self._hold_target, self._yaw, self._yaw_rate,
                                  self.heading_kp, self.heading_kd, self.heading_limit,
                                  self.heading_deadband)

    def _targets(self) -> dict:
        if time.monotonic() - self._cmd_time > self.timeout:
            self._hold_target = None
            self._correction = 0.0
            return {m['wheel']: 0.0 for m in self.motors}
        vx, vy, wz = self._twist
        self._correction = self._update_heading(vx, vy, wz)
        cmd = mecanum(
            vx / self.max_linear if self.max_linear > 0 else 0.0,
            vy / self.max_linear if self.max_linear > 0 else 0.0,
            (wz + self._correction) / self.max_angular if self.max_angular > 0 else 0.0)
        return {m['wheel']: cmd.get(m['wheel'], 0.0) for m in self.motors}

    def _drive(self, motor: dict, value: float):
        if motor['reversed']:
            value = -value
        duty = int(round(min(1.0, abs(value)) * self.max_duty * 4095))
        if duty == 0:
            state = ((0, FULL), (0, FULL), (0, FULL))
        elif value > 0:
            state = ((0, duty), (FULL, 0), (0, FULL))
        else:
            state = ((0, duty), (0, FULL), (FULL, 0))
        if self._written.get(motor['channel']) == state:
            return
        for ch, (on, off) in zip(
                (motor['pwm_channel'], motor['in1_channel'], motor['in2_channel']), state):
            self.pca.set(ch, on, off)
        self._written[motor['channel']] = state

    def _update(self):
        self._connect()
        if not self.pca:
            return
        targets = self._targets()
        try:
            for m in self.motors:
                self._drive(m, targets[m['wheel']])
        except OSError as e:
            self.get_logger().warn(f'Motor HAT 書き込みエラー: {e}')
            self.error = str(e)
            self.pca.close()
            self.pca = None
            self._next_open = time.monotonic() + 1.0

    def _publish_status(self):
        targets = self._targets()
        holding = self._hold_target is not None
        msg = String()
        msg.data = json.dumps({
            'connected': self.pca is not None,
            'error': self.error,
            'output': {k: round(v, 3) for k, v in targets.items()},
            'heading_hold': {
                'enabled': self.heading_hold,
                'active': holding,
                'error_deg': round(math.degrees(_wrap(self._hold_target - self._yaw)), 2)
                if holding else None,
                'correction': round(self._correction, 3),
            },
        }, ensure_ascii=False)
        self.status_pub.publish(msg)

    def stop(self):
        if self.pca:
            try:
                self.pca.all_off()
            except OSError:
                pass
            self.pca.close()
            self.pca = None


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
