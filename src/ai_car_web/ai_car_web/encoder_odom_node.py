#!/usr/bin/env python3
"""4 輪のエンコーダーから車輪のオドメトリ（/odom と odom → base_footprint の TF）を出すノード。

A/B 相の両エッジを GPIO の v2 uAPI のイベントで読み、4 逓倍でカウントする。パルスが多い
（1 輪 1 秒に数千エッジ）ので、ROS の executor と GIL を取り合わないように数えるのは
別プロセス（spawn）で行い、カウントは共有メモリで受け取る。
ピンと車輪の割り付け・向き（reversed）は motor_hat.yaml。
4 輪の進んだ距離からメカナムの順運動学で、前後・左右の速さと旋回を出して積分する。
回った向きは、メカナムのローラーがすべってずれやすいので、IMU（/imu/data、ジャイロ＋磁気）の
yaw の変わった分を使う（yaw_source: imu）。IMU が届かないときや、向きがとんだときは車輪から出す。
"""

import fcntl
import math
import multiprocessing as mp
import os
import select
import signal
import struct
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from ai_car_web.motor_hat import load_motor_hat

# (前の状態, 次の状態) -> 向き。状態 = A * 2 + B
_STEP = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
         (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}

# Linux GPIO character device v2 uAPI（linux/gpio.h）
_GET_CHIPINFO = 0x8044B401        # GPIO_GET_CHIPINFO_IOCTL (struct gpiochip_info 68 B)
_GET_LINE = 0xC250B407            # GPIO_V2_GET_LINE_IOCTL (struct gpio_v2_line_request 592 B)
_GET_VALUES = 0xC010B40E          # GPIO_V2_LINE_GET_VALUES_IOCTL (struct gpio_v2_line_values)
_FLAG_INPUT_BOTH_EDGES = (1 << 2) | (1 << 4) | (1 << 5)
_EVENT_RISING = 1
_EVENT = struct.Struct('=QIIII24x')   # struct gpio_v2_line_event (48 B)
_EVENT_BUFFER = 1024                  # カーネルの上限（GPIO_V2_LINES_MAX × 16）


def _open_chip(name: str) -> int:
    if name:
        return os.open(name if name.startswith('/') else f'/dev/{name}', os.O_RDONLY)
    for index in range(10):
        try:
            fd = os.open(f'/dev/gpiochip{index}', os.O_RDONLY)
        except OSError:
            continue
        info = bytearray(68)
        fcntl.ioctl(fd, _GET_CHIPINFO, info)
        if bytes(info[32:64]).split(b'\0')[0] == b'pinctrl-rp1':
            return fd
        os.close(fd)
    raise OSError('pinctrl-rp1 の gpiochip が見つかりません')


def _request_lines(chip: int, offsets: list) -> int:
    req = bytearray(592)
    struct.pack_into(f'{len(offsets)}I', req, 0, *offsets)
    consumer = b'encoder_odom'
    req[256:256 + len(consumer)] = consumer
    struct.pack_into('Q', req, 288, _FLAG_INPUT_BOTH_EDGES)
    struct.pack_into('II', req, 560, len(offsets), _EVENT_BUFFER)
    fcntl.ioctl(chip, _GET_LINE, req)
    return struct.unpack_from('i', req, 588)[0]


def _get_values(fd: int, n: int) -> list:
    vals = bytearray(16)
    struct.pack_into('QQ', vals, 0, 0, (1 << n) - 1)
    fcntl.ioctl(fd, _GET_VALUES, vals)
    bits = struct.unpack_from('Q', vals, 0)[0]
    return [(bits >> j) & 1 for j in range(n)]


def count_worker(chip_name: str, pins: list, counts, errors, dropped, alive):
    """pins = [(A の BCM, B の BCM), ...] を数え続け、counts / errors に足す。

    libgpiod v1 の Python は 1 本 16 個までしかエッジをためず、1 個ずつ読むのも
    遅いので、エッジを取りこぼす。GPIO の v2 uAPI で 8 本を 1 つの要求にして
    1,024 個までためさせ、まとめて読む。seqno が飛んだ数を dropped に足す。
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        chip = _open_chip(chip_name)
        offsets = [p for pair in pins for p in pair]
        fd = _request_lines(chip, offsets)
        values = _get_values(fd, len(offsets))
    except OSError as e:
        alive.value = -1
        print(f'エンコーダーの GPIO を開けません: {e}', flush=True)
        return
    where = {}
    for i, (a, b) in enumerate(pins):
        where[a] = (i, 0)
        where[b] = (i, 1)
    level = [[values[2 * i], values[2 * i + 1]] for i in range(len(pins))]
    poller = select.poll()
    poller.register(fd, select.POLLIN)
    last_seq = 0
    alive.value = 1
    while alive.value == 1:
        if not poller.poll(100):
            continue
        data = os.read(fd, _EVENT.size * _EVENT_BUFFER)
        for _, kind, offset, seqno, _ in _EVENT.iter_unpack(data):
            if last_seq and seqno != last_seq + 1:
                dropped.value += seqno - last_seq - 1
            last_seq = seqno
            i, bit = where[offset]
            pair = level[i]
            prev = pair[0] * 2 + pair[1]
            pair[bit] = 1 if kind == _EVENT_RISING else 0
            step = _STEP.get((prev, pair[0] * 2 + pair[1]))
            if step is None:
                errors[i] += 1
            else:
                counts[i] += step
    os.close(fd)
    os.close(chip)


class EncoderOdomNode(Node):
    """エンコーダーのカウントから /odom と TF を出す。"""

    def __init__(self):
        super().__init__('encoder_odom_node')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('motor_hat_config', '')
        self.declare_parameter('gpio_chip', '')
        self.declare_parameter('publish_rate', 20.0)
        # 車輪 1 回転のカウント（11 パルス × 4 逓倍 × 減速 56）
        self.declare_parameter('counts_per_rev', 1408.0)
        self.declare_parameter('wheel_radius', 0.0403)
        # 車体の中心から車輪の中心まで（前後 lx・左右 ly）
        self.declare_parameter('wheel_base_half', 0.11)
        self.declare_parameter('track_half', 0.12)
        self.declare_parameter('yaw_scale', 1.0)
        # モーターの正転（Motor HAT の + 出力）でカウントが減るなら -1
        self.declare_parameter('count_sign', -1)
        # 回った向きを imu（/imu/data の yaw の変わった分）と wheel（車輪）のどちらから取るか
        self.declare_parameter('yaw_source', 'imu')
        self.declare_parameter('imu_topic', '/imu/data')
        # この秒数 IMU が来なければ車輪にもどす
        self.declare_parameter('imu_timeout', 0.5)
        # 1 回の変わった分がこの速さ（rad/s）を超えたら、IMU の向きがとんだ（起動・磁気で合わせた）とみて車輪を使う
        self.declare_parameter('imu_max_rate', 4.0)
        # 車輪が 1 カウントも動いていないあいだは回っていないとみる（起動のあと IMU の yaw が磁気の向きへ寄っていく分を入れない）
        self.declare_parameter('imu_still_gate', True)

        path = self.get_parameter('motor_hat_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'motor_hat.yaml')
        cfg = load_motor_hat(path)
        if cfg['error']:
            self.get_logger().error(f'Motor HAT の割り付け: {cfg["error"]}')
        self.wheels = []
        pins = []
        for m in cfg['motors']:
            enc = m.get('encoder') or {}
            a, b = enc.get('a') or {}, enc.get('b') or {}
            if not m['wheel'] or a.get('bcm') is None or b.get('bcm') is None:
                continue
            sign = int(self.get_parameter('count_sign').value) * (-1 if m['reversed'] else 1)
            self.wheels.append((m['wheel'], sign))
            pins.append((a['bcm'], b['bcm']))
        names = {w for w, _ in self.wheels}
        if names != {'front_left', 'front_right', 'rear_left', 'rear_right'}:
            self.get_logger().error(f'エンコーダーが 4 輪そろっていません: {sorted(names)}')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        cpr = float(self.get_parameter('counts_per_rev').value)
        self.m_per_count = 2.0 * math.pi * float(self.get_parameter('wheel_radius').value) / cpr
        self.k = (float(self.get_parameter('wheel_base_half').value)
                  + float(self.get_parameter('track_half').value)) / float(
            self.get_parameter('yaw_scale').value)

        ctx = mp.get_context('spawn')
        self.counts = ctx.Array('q', len(pins), lock=False)
        self.errors = ctx.Array('q', len(pins), lock=False)
        self.dropped = ctx.Value('q', 0, lock=False)
        self.alive = ctx.Value('i', 0, lock=False)
        self.proc = ctx.Process(
            target=count_worker, daemon=True,
            args=(str(self.get_parameter('gpio_chip').value), pins,
                  self.counts, self.errors, self.dropped, self.alive))
        self.proc.start()

        self.x = self.y = self.yaw = 0.0
        self.use_imu = str(self.get_parameter('yaw_source').value) == 'imu'
        self.imu_timeout = float(self.get_parameter('imu_timeout').value)
        self.imu_max_rate = float(self.get_parameter('imu_max_rate').value)
        self.imu_still_gate = bool(self.get_parameter('imu_still_gate').value)
        self._imu_yaw = None
        self._imu_time = None
        self._imu_last_yaw = None
        self._yaw_from_imu = None
        if self.use_imu:
            self.create_subscription(
                Imu, self.get_parameter('imu_topic').value, self._imu_cb, 10)
        self._last = None
        self._last_time = None
        self._errors_logged = 0
        self.odom_pub = self.create_publisher(
            Odometry, self.get_parameter('odom_topic').value, 10)
        self.tf_pub = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / float(self.get_parameter('publish_rate').value), self._update)
        self.create_timer(10.0, self._check)
        self.get_logger().info(
            'エンコーダー: ' + ', '.join(
                f'{w} GPIO{a}/{b}' for (w, _), (a, b) in zip(self.wheels, pins))
            + f'、1 カウント {self.m_per_count * 1000:.3f} mm')

    def _wheel_dist(self) -> dict:
        now = list(self.counts)
        last, self._last = self._last, now
        if last is None:
            return None
        return {w: (n - p) * sign * self.m_per_count
                for (w, sign), n, p in zip(self.wheels, now, last)}

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        self._imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._imu_time = time.monotonic()

    def _choose_dyaw(self, t: float, dt: float, wheel_dyaw: float, still: bool = False) -> float:
        """IMU の yaw が使えればその変わった分、使えなければ車輪の分を返す。still なら 0。"""
        dyaw, from_imu = wheel_dyaw, False
        if self.use_imu:
            yaw = self._imu_yaw
            fresh = yaw is not None and t - self._imu_time <= self.imu_timeout
            last, self._imu_last_yaw = self._imu_last_yaw, (yaw if fresh else None)
            if fresh and last is not None:
                d = math.atan2(math.sin(yaw - last), math.cos(yaw - last))
                if abs(d) <= self.imu_max_rate * dt:
                    dyaw, from_imu = (0.0 if still and self.imu_still_gate else d), True
        if self.use_imu and from_imu != self._yaw_from_imu:
            self.get_logger().info('回った向き: ' + ('IMU' if from_imu else '車輪（IMU が使えない）'))
        self._yaw_from_imu = from_imu
        return dyaw

    def _update(self):
        if self.alive.value != 1 or not rclpy.ok():
            return
        t = time.monotonic()
        d = self._wheel_dist()
        dt = t - self._last_time if self._last_time else 0.0
        self._last_time = t
        if d is None or len(d) != 4 or dt <= 0.0:
            return
        fl, fr = d['front_left'], d['front_right']
        rl, rr = d['rear_left'], d['rear_right']
        dx = (fl + fr + rl + rr) / 4.0
        dy = (-fl + fr + rl - rr) / 4.0
        dyaw = self._choose_dyaw(t, dt, (-fl + fr - rl + rr) / (4.0 * self.k),
                                 still=not any(d.values()))
        mid = self.yaw + dyaw / 2.0
        self.x += dx * math.cos(mid) - dy * math.sin(mid)
        self.y += dx * math.sin(mid) + dy * math.cos(mid)
        self.yaw = math.atan2(math.sin(self.yaw + dyaw), math.cos(self.yaw + dyaw))

        stamp = self.get_clock().now().to_msg()
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = dx / dt
        odom.twist.twist.linear.y = dy / dt
        odom.twist.twist.angular.z = dyaw / dt
        self.odom_pub.publish(odom)
        if self.tf_pub:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_pub.sendTransform(tf)

    def _check(self):
        if self.alive.value == -1 or not self.proc.is_alive():
            self.get_logger().error('エンコーダーを数えるプロセスが止まっています')
            return
        total = sum(self.errors) + self.dropped.value
        if total > self._errors_logged:
            names = [w for w, _ in self.wheels]
            self.get_logger().warn(
                f'エンコーダーの読みとばし: {dict(zip(names, self.errors))}'
                f'、取りこぼし {self.dropped.value}'
                f'（カウント {dict(zip(names, self.counts))}）')
            self._errors_logged = total

    def stop(self):
        self.alive.value = 0
        self.proc.join(timeout=1.0)
        if self.proc.is_alive():
            self.proc.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdomNode()
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
