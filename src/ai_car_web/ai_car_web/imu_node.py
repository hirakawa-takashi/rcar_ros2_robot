#!/usr/bin/env python3
"""GY-BNO055 を読み取り sensor_msgs/Imu として publish するノード。"""

import fcntl
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

I2C_SLAVE = 0x0703
CHIP_ID = 0x00
PAGE_ID = 0x07
UNIT_SEL = 0x3B
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
AXIS_MAP_CONFIG = 0x41
AXIS_MAP_SIGN = 0x42
CALIB_OFFSETS = 0x55
CALIB_SIZE = 22
CONFIG_MODE = 0x00
AMG_MODE = 0x07
IMU_MODE = 0x08
NDOF_MODE = 0x0C
GRAVITY = 9.80665


class ImuNode(Node):
    """GY-BNO055 を I2C から読み取り、IMU メッセージを publish する。"""

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x29)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('axis_map_config', 0x24)
        self.declare_parameter('axis_map_sign', 0x00)
        self.declare_parameter('operation_mode', NDOF_MODE)
        self.declare_parameter('calibration_file', '~/.ros/bno055_calibration.json')
        self.declare_parameter('calibration_save_interval', 600.0)
        self.declare_parameter('filter_gain', 1.0)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        mag_topic = str(self.get_parameter('mag_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.axis_map_config = int(self.get_parameter('axis_map_config').value)
        self.axis_map_sign = int(self.get_parameter('axis_map_sign').value)
        self.operation_mode = int(self.get_parameter('operation_mode').value)
        self.calibration_file = os.path.expanduser(
            str(self.get_parameter('calibration_file').value))
        self.calibration_save_interval = float(
            self.get_parameter('calibration_save_interval').value)
        self.filter_gain = float(self.get_parameter('filter_gain').value)

        self.pub = self.create_publisher(Imu, imu_topic, 10)
        self.mag_pub = (self.create_publisher(MagneticField, mag_topic, 10)
                        if mag_topic and self.operation_mode != IMU_MODE else None)
        self._q = None
        self._q_time = None
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._fd = None
        self._next_retry = 0.0
        self._connection_warned = False
        self._last_error_log = 0.0
        self._calibration = None
        self._temperature_c = None
        self._calib_saved_at = None
        self.create_timer(1.0 / max(publish_rate, 1.0), self._timer_cb)

    def _write_reg(self, reg: int, value: int):
        if os.write(self._fd, bytes([reg, value])) != 2:
            raise OSError(f'I2C レジスタ書き込み失敗: 0x{reg:02x}')

    def _read_reg(self, reg: int, size: int) -> bytes:
        if os.write(self._fd, bytes([reg])) != 1:
            raise OSError(f'I2C レジスタ指定失敗: 0x{reg:02x}')
        data = os.read(self._fd, size)
        if len(data) != size:
            raise OSError(f'I2C 読み取り長不足: 0x{reg:02x}')
        return data

    def _initialize_sensor(self):
        if self._read_reg(CHIP_ID, 1)[0] != 0xA0:
            raise OSError('BNO055 CHIP_ID が不正です')
        self._write_reg(PAGE_ID, 0x00)
        self._write_reg(OPR_MODE, CONFIG_MODE)
        time.sleep(0.03)
        self._write_reg(SYS_TRIGGER, 0x20)
        time.sleep(0.7)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if self._read_reg(CHIP_ID, 1)[0] == 0xA0:
                break
            time.sleep(0.05)
        else:
            raise OSError(
                'BNO055 リセット後の CHIP_ID 待機がタイムアウトしました')
        self._write_reg(PWR_MODE, 0x00)
        self._write_reg(UNIT_SEL, 0x00)
        self._write_reg(SYS_TRIGGER, 0x00)
        self._write_reg(AXIS_MAP_CONFIG, self.axis_map_config)
        self._write_reg(AXIS_MAP_SIGN, self.axis_map_sign)
        self._load_calibration()
        self._write_reg(OPR_MODE, self.operation_mode)
        time.sleep(0.03)

    def _load_calibration(self):
        try:
            with open(self.calibration_file, encoding='utf-8') as f:
                offsets = bytes(json.load(f)['offsets'])
        except FileNotFoundError:
            return
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'BNO055 の補正ファイルを読めません: {e}')
            return
        if len(offsets) != CALIB_SIZE:
            self.get_logger().warn('BNO055 の補正ファイルの長さが不正です')
            return
        if os.write(self._fd, bytes([CALIB_OFFSETS]) + offsets) != CALIB_SIZE + 1:
            raise OSError('BNO055 補正値の書き込み失敗')
        self.get_logger().info(f'BNO055 の補正を読みこみました: {self.calibration_file}')

    def _calibration_done(self) -> bool:
        if self.operation_mode == AMG_MODE:
            return False
        if self.operation_mode == IMU_MODE:
            return self._calibration[1:3] == (3, 3)
        return self._calibration[1:] == (3, 3, 3)

    def _save_calibration(self, now: float):
        self._calib_saved_at = now
        self._write_reg(OPR_MODE, CONFIG_MODE)
        time.sleep(0.025)
        try:
            offsets = self._read_reg(CALIB_OFFSETS, CALIB_SIZE)
        finally:
            self._write_reg(OPR_MODE, self.operation_mode)
            time.sleep(0.02)
        tmp = self.calibration_file + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.calibration_file) or '.', exist_ok=True)
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump({'saved': time.strftime('%Y-%m-%dT%H:%M:%S'),
                           'calibration': list(self._calibration),
                           'offsets': list(offsets)}, f)
            os.replace(tmp, self.calibration_file)
        except OSError as e:
            self.get_logger().warn(f'BNO055 の補正を保存できません: {e}')
            return
        self.get_logger().info(f'BNO055 の補正を保存しました: {self.calibration_file}')

    def _connect(self):
        fd = os.open(f'/dev/i2c-{self.i2c_bus}', os.O_RDWR)
        try:
            fcntl.ioctl(fd, I2C_SLAVE, self.i2c_address)
            self._fd = fd
            self._initialize_sensor()
        except OSError:
            self._fd = None
            try:
                os.close(fd)
            except OSError:
                pass
            raise
        self._calibration = None
        self._q = None
        self._connection_warned = False
        self.get_logger().info(
            f'BNO055 接続: /dev/i2c-{self.i2c_bus}, 0x{self.i2c_address:02x}')

    def _disconnect(self):
        fd = self._fd
        self._fd = None
        self._calibration = None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _warn_error(self, message: str, error: OSError):
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            self.get_logger().warn(f'{message}: {error}')
            self._last_error_log = now

    @staticmethod
    def _i16(data: bytes, offset: int) -> int:
        return int.from_bytes(data[offset:offset + 2], 'little', signed=True)

    def _publish_data(self, data: bytes):
        acc = [self._i16(data, off) / 100.0 for off in (0, 2, 4)]
        mag_ut = [self._i16(data, off) / 16.0 for off in (6, 8, 10)]
        gyr = [self._i16(data, off) / 16.0 * math.pi / 180.0
               for off in (12, 14, 16)]
        self._temperature_c = int.from_bytes(data[44:45], 'little', signed=True)
        stamp = self.get_clock().now().to_msg()
        if self.operation_mode == AMG_MODE:
            gyr = self._remove_gyro_bias(gyr, acc)
            quat_wxyz = self._update_orientation(gyr, acc)
        else:
            quat_wxyz = [self._i16(data, off) / 16384.0
                         for off in (24, 26, 28, 30)]
            self._log_calibration(data[45])
        if self.mag_pub is not None:
            mag = MagneticField()
            mag.header.stamp = stamp
            mag.header.frame_id = self.frame_id
            mag.magnetic_field.x = mag_ut[0] * 1e-6
            mag.magnetic_field.y = mag_ut[1] * 1e-6
            mag.magnetic_field.z = mag_ut[2] * 1e-6
            self.mag_pub.publish(mag)

        msg = Imu()
        msg.header.stamp = stamp
        self._publish_imu(msg, acc, gyr, quat_wxyz)

    def _log_calibration(self, stat: int):
        calib = (
            (stat >> 6) & 0x03,
            (stat >> 4) & 0x03,
            (stat >> 2) & 0x03,
            stat & 0x03,
        )
        if calib != self._calibration:
            self._calibration = calib
            self.get_logger().info(
                f'BNO055 キャリブレーション: sys={calib[0]} gyr={calib[1]} '
                f'acc={calib[2]} mag={calib[3]}')

    def _remove_gyro_bias(self, gyr: list, acc: list) -> list:
        still = (abs(math.sqrt(sum(a * a for a in acc)) - GRAVITY) < 0.5
                 and all(abs(g - b) < 0.05 for g, b in zip(gyr, self._gyro_bias)))
        if still:
            self._gyro_bias = [b + 0.05 * (g - b) for g, b in zip(gyr, self._gyro_bias)]
        return [g - b for g, b in zip(gyr, self._gyro_bias)]

    def _update_orientation(self, gyr: list, acc: list) -> list:
        now = time.monotonic()
        norm = math.sqrt(sum(a * a for a in acc))
        if self._q is None:
            if norm < 1e-3:
                return [1.0, 0.0, 0.0, 0.0]
            roll = math.atan2(acc[1], acc[2])
            pitch = math.atan2(-acc[0], math.hypot(acc[1], acc[2]))
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            self._q = [cr * cp, sr * cp, cr * sp, -sr * sp]
            self._q_time = now
            return list(self._q)
        dt = min(max(now - self._q_time, 0.0), 0.2)
        self._q_time = now
        w, x, y, z = self._q
        gx, gy, gz = gyr
        if abs(norm - GRAVITY) < 2.0:
            ax, ay, az = (a / norm for a in acc)
            vx = 2.0 * (x * z - w * y)
            vy = 2.0 * (w * x + y * z)
            vz = w * w - x * x - y * y + z * z
            gx += self.filter_gain * (ay * vz - az * vy)
            gy += self.filter_gain * (az * vx - ax * vz)
            gz += self.filter_gain * (ax * vy - ay * vx)
        h = 0.5 * dt
        w, x, y, z = (w - h * (x * gx + y * gy + z * gz),
                      x + h * (w * gx + y * gz - z * gy),
                      y + h * (w * gy - x * gz + z * gx),
                      z + h * (w * gz + x * gy - y * gx))
        n = math.sqrt(w * w + x * x + y * y + z * z)
        self._q = [w / n, x / n, y / n, z / n]
        return list(self._q)

    def _publish_imu(self, msg: Imu, acc: list, gyr: list, quat_wxyz: list):
        msg.header.frame_id = self.frame_id
        msg.orientation.x = quat_wxyz[1]
        msg.orientation.y = quat_wxyz[2]
        msg.orientation.z = quat_wxyz[3]
        msg.orientation.w = quat_wxyz[0]
        msg.angular_velocity.x = gyr[0]
        msg.angular_velocity.y = gyr[1]
        msg.angular_velocity.z = gyr[2]
        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001
        msg.linear_acceleration_covariance[0] = 0.05
        msg.linear_acceleration_covariance[4] = 0.05
        msg.linear_acceleration_covariance[8] = 0.05
        self.pub.publish(msg)

    def _timer_cb(self):
        now = time.monotonic()
        if self._fd is None:
            if now < self._next_retry:
                return
            try:
                self._connect()
            except OSError as e:
                if not self._connection_warned:
                    self.get_logger().warn(f'BNO055 に接続できません: {e}')
                    self._connection_warned = True
                self._next_retry = now + 2.0
                return
        try:
            mode = self._read_reg(OPR_MODE, 1)[0]
            if mode != self.operation_mode:
                self.get_logger().warn(
                    f'BNO055 の動作モードが 0x{mode:02x} になった（リセットされた）ので設定し直します')
                self._disconnect()
                self._next_retry = now
                return
            data = self._read_reg(0x08, 46)
            self._publish_data(data)
            if (self._calibration_done()
                    and (self._calib_saved_at is None
                         or now - self._calib_saved_at >= self.calibration_save_interval)):
                self._save_calibration(now)
        except OSError as e:
            self._warn_error('BNO055 読み取りエラー', e)
            self._disconnect()
            self._next_retry = time.monotonic() + 2.0

    def destroy_node(self):
        self._disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
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
