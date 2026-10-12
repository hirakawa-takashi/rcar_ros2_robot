#!/usr/bin/env python3
"""4 輪のエンコーダーから車輪のオドメトリ（/odom と odom → base_footprint の TF）を出すノード。

A/B 相の両エッジを libgpiod のイベントで読み、4 逓倍でカウントする。パルスが多い
（1 輪 1 秒に数千エッジ）ので、ROS の executor と GIL を取り合わないように数えるのは
別プロセス（spawn）で行い、カウントは共有メモリで受け取る。
ピンと車輪の割り付け・向き（reversed）は motor_hat.yaml。
4 輪の進んだ距離からメカナムの順運動学で、前後・左右の速さと旋回を出して積分する。
"""

import math
import multiprocessing as mp
import os
import signal
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from ai_car_web.motor_hat import load_motor_hat

try:
    import gpiod
except ImportError:  # pragma: no cover - Pi 以外
    gpiod = None

# (前の状態, 次の状態) -> 向き。状態 = A * 2 + B
_STEP = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1,
         (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}


def _find_chip(name: str):
    if name:
        return gpiod.Chip(name)
    for index in range(10):
        try:
            chip = gpiod.Chip(f'/dev/gpiochip{index}')
        except OSError:
            continue
        if chip.label() == 'pinctrl-rp1':
            return chip
        chip.close()
    raise OSError('pinctrl-rp1 の gpiochip が見つかりません')


def count_worker(chip_name: str, pins: list, counts, errors, alive):
    """pins = [(A の BCM, B の BCM), ...] を数え続け、counts / errors に足す。"""
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        chip = _find_chip(chip_name)
        offsets = [p for pair in pins for p in pair]
        lines = chip.get_lines(offsets)
        lines.request(consumer='encoder_odom', type=gpiod.LINE_REQ_EV_BOTH_EDGES)
    except OSError as e:
        alive.value = -1
        print(f'エンコーダーの GPIO を開けません: {e}', flush=True)
        return
    where = {}
    for i, (a, b) in enumerate(pins):
        where[a] = (i, 1)
        where[b] = (i, 0)
    values = lines.get_values()
    level = [[values[2 * i], values[2 * i + 1]] for i in range(len(pins))]
    alive.value = 1
    rising = gpiod.LineEvent.RISING_EDGE
    while alive.value == 1:
        ready = lines.event_wait(sec=0, nsec=100_000_000)
        if not ready:
            continue
        events = []
        for line in ready:
            events.extend(line.event_read_multiple())
        events.sort(key=lambda e: (e.sec, e.nsec))
        for ev in events:
            i, bit = where[ev.source.offset()]
            pair = level[i]
            prev = pair[0] * 2 + pair[1]
            pair[1 - bit] = 1 if ev.type == rising else 0
            step = _STEP.get((prev, pair[0] * 2 + pair[1]))
            if step is None:
                errors[i] += 1
            else:
                counts[i] += step
    lines.release()
    chip.close()


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
        self.declare_parameter('counts_per_rev', 2464.0)
        self.declare_parameter('wheel_radius', 0.0403)
        # 車体の中心から車輪の中心まで（前後 lx・左右 ly）
        self.declare_parameter('wheel_base_half', 0.11)
        self.declare_parameter('track_half', 0.12)
        # モーターの正転（Motor HAT の + 出力）でカウントが減るなら -1
        self.declare_parameter('count_sign', -1)

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
                  + float(self.get_parameter('track_half').value))

        ctx = mp.get_context('spawn')
        self.counts = ctx.Array('q', len(pins), lock=False)
        self.errors = ctx.Array('q', len(pins), lock=False)
        self.alive = ctx.Value('i', 0, lock=False)
        self.proc = ctx.Process(
            target=count_worker, daemon=True,
            args=(str(self.get_parameter('gpio_chip').value), pins,
                  self.counts, self.errors, self.alive))
        self.proc.start()

        self.x = self.y = self.yaw = 0.0
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
        dyaw = (-fl + fr - rl + rr) / (4.0 * self.k)
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
        total = sum(self.errors)
        if total > self._errors_logged:
            names = [w for w, _ in self.wheels]
            self.get_logger().warn(
                f'エンコーダーの読みとばし: {dict(zip(names, self.errors))}'
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
