#!/usr/bin/env python3
"""GY-BNO055 を読み取り sensor_msgs/Imu として publish するノード。"""

import fcntl
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

I2C_SLAVE = 0x0703
CHIP_ID = 0x00
PAGE_ID = 0x07
UNIT_SEL = 0x3B
OPR_MODE = 0x3D
PWR_MODE = 0x3E
SYS_TRIGGER = 0x3F
CONFIG_MODE = 0x00
NDOF_MODE = 0x0C


class ImuNode(Node):
    """GY-BNO055 を I2C から読み取り、IMU メッセージを publish する。"""

    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x29)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('publish_rate', 50.0)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.i2c_address = int(self.get_parameter('i2c_address').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        self.pub = self.create_publisher(Imu, imu_topic, 10)
        self._fd = None
        self._next_retry = 0.0
        self._connection_warned = False
        self._last_error_log = 0.0
        self._calibration = None
        self._temperature_c = None
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
        self._write_reg(OPR_MODE, NDOF_MODE)
        time.sleep(0.03)

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
        gyr = [self._i16(data, off) / 16.0 * math.pi / 180.0
               for off in (12, 14, 16)]
        quat_wxyz = [self._i16(data, off) / 16384.0
                     for off in (24, 26, 28, 30)]
        self._temperature_c = int.from_bytes(data[44:45], 'little', signed=True)
        calib = (
            (data[45] >> 6) & 0x03,
            (data[45] >> 4) & 0x03,
            (data[45] >> 2) & 0x03,
            data[45] & 0x03,
        )
        if calib != self._calibration:
            self._calibration = calib
            self.get_logger().info(
                f'BNO055 キャリブレーション: sys={calib[0]} gyr={calib[1]} '
                f'acc={calib[2]} mag={calib[3]}')

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
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
            data = self._read_reg(0x08, 46)
            self._publish_data(data)
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
