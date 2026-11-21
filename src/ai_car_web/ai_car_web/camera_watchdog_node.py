#!/usr/bin/env python3
"""カメラ映像が途絶えたら camera_node を終了させ、launch の respawn で起動し直させるノード。

camera_ros（libcamera）は、動いたまま /camera/image_raw/compressed を出さなくなる。
プロセスは生きたまま respawn されないので、ダッシュボードの映像だけが消える
（2026-11-20 実機：起動から約 7 時間で request cancelled が出て、約 14 時間戻らなかった）。
image_timeout 秒そのトピックが来なければ camera_node に SIGTERM を送り、
kill_wait 秒たっても残っていれば SIGKILL を送る。起動直後と起動し直した後は
startup_grace 秒待ってから監視する。

libcamera が request を取り違える状態（request cancelled）は、camera_node を
起こすときれることが多い。起こし直しが warn_after 回続いて映像が戻らないときは、
起こす間隔を retry_gap 秒あけて、くり返し起こしてログを増やさないようにする
（止まった camera_node は 1 秒に約 200 行 out する。14 時間で 13 GB のログになった）。
"""

import os
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def find_pids(pattern: str):
    pids = []
    for name in os.listdir('/proc'):
        if not name.isdigit() or int(name) == os.getpid():
            continue
        try:
            with open(f'/proc/{name}/cmdline', 'rb') as f:
                cmdline = f.read().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            continue
        if pattern in cmdline:
            pids.append(int(name))
    return pids


class CameraWatchdogNode(Node):
    def __init__(self):
        super().__init__('camera_watchdog_node')
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('image_timeout', 20.0)
        self.declare_parameter('startup_grace', 20.0)
        self.declare_parameter('kill_wait', 3.0)
        self.declare_parameter('process_pattern', 'camera_ros/camera_node')
        self.declare_parameter('warn_after', 3)
        self.declare_parameter('retry_gap', 300.0)

        self.image_timeout = float(self.get_parameter('image_timeout').value)
        self.startup_grace = float(self.get_parameter('startup_grace').value)
        self.kill_wait = float(self.get_parameter('kill_wait').value)
        self.pattern = str(self.get_parameter('process_pattern').value)
        self.warn_after = int(self.get_parameter('warn_after').value)
        self.retry_gap = float(self.get_parameter('retry_gap').value)

        self._last_frame = None
        self._watch_from = time.monotonic() + self.startup_grace
        self._term_pids = []
        self._term_at = None
        self._restarts = 0
        self._fails = 0
        self._next_restart_at = 0.0

        self.create_subscription(
            CompressedImage, str(self.get_parameter('image_topic').value),
            self._frame_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)

    def _frame_cb(self, _msg):
        self._last_frame = time.monotonic()
        self._fails = 0
        self._next_restart_at = 0.0

    def _tick(self):
        now = time.monotonic()
        if self._term_at is not None:
            if now - self._term_at < self.kill_wait:
                return
            alive = [p for p in self._term_pids if p in find_pids(self.pattern)]
            for pid in alive:
                self.get_logger().warn(f'camera_node（pid {pid}）が終わらないので SIGKILL を送ります')
                self._signal(pid, signal.SIGKILL)
            self._term_pids, self._term_at = [], None
            self._last_frame = None
            self._watch_from = now + self.startup_grace
            return
        if now < self._watch_from:
            return
        age = now - max(self._last_frame or 0.0, self._watch_from)
        if age < self.image_timeout:
            return
        pids = find_pids(self.pattern)
        if not pids:
            # camera_node がまだ起動していない（respawn 待ち・use_camera:=false）
            self._watch_from = now + self.startup_grace
            return
        if now < self._next_restart_at:
            return
        self._restarts += 1
        self._fails += 1
        self.get_logger().warn(
            f'カメラ映像が {age:.1f} 秒来ないので camera_node を起動し直します'
            f'（{self._restarts} 回目）')
        if 0 < self.warn_after <= self._fails:
            self.get_logger().error(
                f'camera_node を {self._fails} 回起こしても映像が戻りません。'
                f'~/opt/rpicam の libcamera と camera_ros の soname、'
                f'CMA の空き、~/.ros/log の camera_node_*.log の大きさを見てください。'
                f'次は {self.retry_gap:.0f} 秒あけて起こします')
            self._next_restart_at = now + self.retry_gap
            self._fails = 0
        for pid in pids:
            self._signal(pid, signal.SIGTERM)
        self._term_pids, self._term_at = pids, now

    def _signal(self, pid, sig):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as e:
            self.get_logger().error(f'camera_node（pid {pid}）を止められません: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraWatchdogNode()
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
