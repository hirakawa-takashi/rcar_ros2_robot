#!/usr/bin/env python3
"""/scan が途絶えたら rplidar ノードを終了させ、launch の respawn で起動し直させるノード。

rplidar_ros は USB（CP2102）が抜けて差し直されると、古いシリアルポートを
握ったまま /scan を出さなくなり、プロセスは生きたままになる。
scan_timeout 秒 /scan が来なければ rplidar_composition に SIGTERM を送り、
kill_wait 秒たっても残っていれば SIGKILL を送る。起動直後と再起動の後は
startup_grace 秒待ってから監視する。

電池の電圧が下がった後などに CP2102 が返事をしなくなると、起動し直しても
/scan は戻らない。usb_reset_after 回続けて戻らなければ、rplidar を止めた後に
LiDAR の USB をつなぎ直す（USBDEVFS_RESET。/dev/bus/usb の書き込み権限は udev ルールで付ける）。

CP2102 が固まると USBDEVFS_RESET でも戻らない（2026-10-08 は約 10 時間 45 分）。
power_cycle_after 回続けば、USB の電気を切って入れ直す（power_cycle_cmd、power_cycles 回まで）。
reboot_after 回続けば、ラズパイを再起動する（reboot_cmd）。再起動は、この起動で /scan が
一度でも来ていて、起動から reboot_min_uptime 秒たっているときだけ（LiDAR がないときや、
再起動しても直らないときに、くり返さない）。
"""

import fcntl
import glob
import os
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

USBDEVFS_RESET = ord('U') << 8 | 20


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


def _read(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def find_usb_device(vendor: str, product: str):
    """idVendor / idProduct が合う USB 機器の /dev/bus/usb のパス。なければ None。"""
    for d in sorted(glob.glob('/sys/bus/usb/devices/*')):
        if _read(f'{d}/idVendor') != vendor or _read(f'{d}/idProduct') != product:
            continue
        bus, dev = _read(f'{d}/busnum'), _read(f'{d}/devnum')
        if bus and dev:
            return f'/dev/bus/usb/{int(bus):03d}/{int(dev):03d}'
    return None


def uptime_seconds():
    try:
        with open('/proc/uptime') as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return 0.0


def reset_usb(path: str):
    fd = os.open(path, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    finally:
        os.close(fd)


class LidarWatchdogNode(Node):
    def __init__(self):
        super().__init__('lidar_watchdog_node')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout', 3.0)
        self.declare_parameter('startup_grace', 15.0)
        self.declare_parameter('kill_wait', 3.0)
        self.declare_parameter('process_pattern', 'rplidar_ros/rplidar_composition')
        self.declare_parameter('usb_reset_after', 2)
        self.declare_parameter('usb_vendor_id', '10c4')
        self.declare_parameter('usb_product_id', 'ea60')
        self.declare_parameter('power_cycle_after', 4)
        self.declare_parameter('power_cycles', 2)
        self.declare_parameter('power_cycle_cmd', ['sudo', '-n', '/usr/local/sbin/ai-car-usb-power-cycle'])
        self.declare_parameter('reboot_after', 7)
        self.declare_parameter('reboot_min_uptime', 1800.0)
        self.declare_parameter('reboot_cmd', ['sudo', '-n', '/usr/bin/systemctl', 'reboot'])

        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.startup_grace = float(self.get_parameter('startup_grace').value)
        self.kill_wait = float(self.get_parameter('kill_wait').value)
        self.pattern = str(self.get_parameter('process_pattern').value)
        self.usb_reset_after = int(self.get_parameter('usb_reset_after').value)
        self.usb_id = (str(self.get_parameter('usb_vendor_id').value),
                       str(self.get_parameter('usb_product_id').value))
        self.power_cycle_after = int(self.get_parameter('power_cycle_after').value)
        self.power_cycles = int(self.get_parameter('power_cycles').value)
        self.power_cycle_cmd = list(self.get_parameter('power_cycle_cmd').value)
        self.reboot_after = int(self.get_parameter('reboot_after').value)
        self.reboot_min_uptime = float(self.get_parameter('reboot_min_uptime').value)
        self.reboot_cmd = list(self.get_parameter('reboot_cmd').value)

        self._last_scan = None
        self._watch_from = time.monotonic() + self.startup_grace
        self._term_pids = []
        self._term_at = None
        self._restarts = 0
        self._fails = 0
        self._scan_seen = False
        self._rebooting = False

        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._scan_cb, qos_profile_sensor_data)
        self.create_timer(0.5, self._tick)

    def _scan_cb(self, _msg):
        self._last_scan = time.monotonic()
        self._fails = 0
        self._scan_seen = True

    def _tick(self):
        now = time.monotonic()
        if self._term_at is not None:
            if now - self._term_at < self.kill_wait:
                return
            alive = [p for p in self._term_pids if p in find_pids(self.pattern)]
            for pid in alive:
                self.get_logger().warn(f'rplidar（pid {pid}）が終わらないので SIGKILL を送ります')
                if self._fails <= 1:
                    self.get_logger().warn(f'rplidar（pid {pid}）のスレッドの待ち先: {thread_waits(pid)}')
                self._signal(pid, signal.SIGKILL)
            self._recover()
            self._term_pids, self._term_at = [], None
            self._last_scan = None
            self._watch_from = now + self.startup_grace
            return
        if now < self._watch_from:
            return
        age = now - max(self._last_scan or 0.0, self._watch_from)
        if age < self.scan_timeout:
            return
        pids = find_pids(self.pattern)
        if not pids:
            self._watch_from = now + self.startup_grace
            return
        self._restarts += 1
        self._fails += 1
        self.get_logger().warn(
            f'/scan が {age:.1f} 秒来ないので rplidar を起動し直します（{self._restarts} 回目）')
        for pid in pids:
            self._signal(pid, signal.SIGTERM)
        self._term_pids, self._term_at = pids, now

    def _recover(self):
        if 0 < self.reboot_after <= self._fails and self._reboot():
            return
        if (0 < self.power_cycle_after <= self._fails < self.power_cycle_after + self.power_cycles
                and self._power_cycle()):
            return
        if 0 < self.usb_reset_after <= self._fails:
            self._reset_usb()

    def _run(self, cmd, what):
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=20)
        except (OSError, subprocess.TimeoutExpired) as e:
            self.get_logger().error(f'{what}できません: {e}')
            return False
        if r.returncode != 0:
            self.get_logger().error(
                f'{what}できません（{r.returncode}）: {(r.stderr or r.stdout).strip()[:200]}')
            return False
        return True

    def _power_cycle(self):
        if find_usb_device(*self.usb_id) is None:
            return False
        self.get_logger().warn(
            f'起動し直しても /scan が {self._fails} 回戻らないので、USB の電気を切って入れ直します'
            '（ゲームパッドも数秒切れます）')
        return self._run(self.power_cycle_cmd, 'USB の電気を切って入れ直し')

    def _reboot(self):
        if self._rebooting:
            return True
        if not self._scan_seen or uptime_seconds() < self.reboot_min_uptime:
            return False
        self.get_logger().error(
            f'USB の電気を入れ直しても /scan が {self._fails} 回戻らないので、ラズパイを再起動します')
        self._rebooting = self._run(self.reboot_cmd, 'ラズパイを再起動')
        return self._rebooting

    def _reset_usb(self):
        path = find_usb_device(*self.usb_id)
        if path is None:
            self.get_logger().error(f'LiDAR の USB（{":".join(self.usb_id)}）が見つかりません')
            return
        try:
            reset_usb(path)
        except OSError as e:
            self.get_logger().error(f'LiDAR の USB（{path}）をつなぎ直せません: {e}')
            return
        self.get_logger().warn(
            f'起動し直しても /scan が {self._fails} 回戻らないので、LiDAR の USB（{path}）をつなぎ直しました')

    def _signal(self, pid, sig):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as e:
            self.get_logger().error(f'rplidar（pid {pid}）を止められません: {e}')


def thread_waits(pid):
    """プロセスの各スレッドが kernel のどこで待っているか（wchan）を「名前:待ち先×数」にする。"""
    counts = {}
    for task in sorted(glob.glob(f'/proc/{pid}/task/*')):
        try:
            with open(f'{task}/comm') as f:
                name = f.read().strip()
            with open(f'{task}/wchan') as f:
                wchan = f.read().strip() or '-'
        except OSError:
            continue
        key = f'{name}:{wchan}'
        counts[key] = counts.get(key, 0) + 1
    return ', '.join(f'{k}×{n}' for k, n in counts.items()) or '不明'


def main(args=None):
    rclpy.init(args=args)
    node = LidarWatchdogNode()
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
