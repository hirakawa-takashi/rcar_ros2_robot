"""Raspberry Pi 5 と AI HAT+ (Hailo-8) の状態を収集して publish するノード。

CPU使用率・温度・メモリ・電源（PMIC ADC）・スロットリング状態と、
AI HAT+ の検出状況およびランタイム情報を JSON 文字列として
`/system_status` (std_msgs/String) に配信する。
"""

import ctypes
import glob
import json
import os
import re
import shutil
import subprocess

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# vcgencmd pmic_read_adc の出力例: "   3V3_SYS_A current(1)=0.09954486A"
_ADC_LINE = re.compile(r'^\s*(?P<rail>\S+)_(?P<kind>[AV])\s+\S+\(\d+\)=(?P<value>[-\d.]+)[AV]$')

# vcgencmd get_throttled のビット定義
_THROTTLE_BITS = {
    0: 'under_voltage_now',
    1: 'arm_freq_capped_now',
    2: 'throttled_now',
    3: 'soft_temp_limit_now',
    16: 'under_voltage_occurred',
    17: 'arm_freq_capped_occurred',
    18: 'throttled_occurred',
    19: 'soft_temp_limit_occurred',
}


_HAILO_DRIVER_SYSFS = '/sys/bus/pci/drivers/hailo'


def _hailo_pci_sysfs():
    """hailo_pci ドライバにバインドされた PCIe デバイスの sysfs パスを返す。"""
    for name in sorted(glob.glob(f'{_HAILO_DRIVER_SYSFS}/[0-9a-f]*:*')):
        return name
    return None


class _HailoChipTemperature(ctypes.Structure):
    """hailo_chip_temperature_info_t (HailoRT C API)。"""

    _fields_ = [
        ('ts0', ctypes.c_float),
        ('ts1', ctypes.c_float),
        ('sample_count', ctypes.c_uint16),
    ]


def _hailo_temperature():
    """libhailort 経由で Hailo-8 のオンチップ温度 [℃] を読む。

    hailortcli 4.24 には温度取得サブコマンドが無いため C API を直接呼ぶ。
    取得できない場合は None。
    """
    try:
        lib = ctypes.CDLL('libhailort.so')
    except OSError:
        return None
    device = ctypes.c_void_p()
    if lib.hailo_create_device_by_id(None, ctypes.byref(device)) != 0:
        return None
    try:
        info = _HailoChipTemperature()
        if lib.hailo_get_chip_temperature(device, ctypes.byref(info)) != 0:
            return None
        return round((info.ts0 + info.ts1) / 2.0, 1)
    finally:
        lib.hailo_release_device(device)


def _read_text(path):
    if path is None:
        return None
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except OSError:
        return None


def _run(cmd, timeout=3.0):
    """コマンドを実行し標準出力を返す。失敗時は None。"""
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=timeout, check=False)
    except (OSError, subprocess.SubprocessError):
        return None
    if result.returncode != 0:
        return None
    return result.stdout.strip()


class SystemMonitorNode(Node):
    """システム情報収集ノード。"""

    def __init__(self):
        super().__init__('system_monitor_node')

        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('topic', '/system_status')
        self.declare_parameter('enable_pmic', True)
        self.declare_parameter('enable_hailo', True)

        rate = float(self.get_parameter('publish_rate').value)
        self.enable_pmic = bool(self.get_parameter('enable_pmic').value)
        self.enable_hailo = bool(self.get_parameter('enable_hailo').value)

        self.has_vcgencmd = shutil.which('vcgencmd') is not None
        self.hailortcli = shutil.which('hailortcli')

        self.pub = self.create_publisher(
            String, self.get_parameter('topic').value, 10)

        psutil.cpu_percent(percpu=True)  # 初回呼び出しで計測を開始する
        self._hailo_static = self._detect_hailo()
        self.create_timer(1.0 / max(rate, 0.1), self._publish_cb)

    # --- CPU / メモリ ---
    def _cpu(self):
        freq = psutil.cpu_freq()
        temp = self._cpu_temperature()
        try:
            load1, load5, load15 = psutil.getloadavg()
        except OSError:
            load1 = load5 = load15 = None
        return {
            'percent': round(psutil.cpu_percent(), 1),
            'per_core': [round(v, 1) for v in psutil.cpu_percent(percpu=True)],
            'freq_mhz': round(freq.current, 0) if freq else None,
            'temperature_c': temp,
            'load_average': [load1, load5, load15],
        }

    def _cpu_temperature(self):
        if self.has_vcgencmd:
            out = _run(['vcgencmd', 'measure_temp'])
            if out and '=' in out:
                try:
                    return round(float(out.split('=')[1].rstrip("'C")), 1)
                except ValueError:
                    pass
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                return round(int(f.read().strip()) / 1000.0, 1)
        except (OSError, ValueError):
            return None

    @staticmethod
    def _memory():
        vm = psutil.virtual_memory()
        sw = psutil.swap_memory()
        disk = psutil.disk_usage('/')
        return {
            'used_mb': round(vm.used / 1024 / 1024, 1),
            'total_mb': round(vm.total / 1024 / 1024, 1),
            'percent': vm.percent,
            'swap_percent': sw.percent,
            'disk_used_gb': round(disk.used / 1024 / 1024 / 1024, 2),
            'disk_total_gb': round(disk.total / 1024 / 1024 / 1024, 2),
            'disk_percent': disk.percent,
        }

    # --- 電源 ---
    def _power(self):
        """PMIC ADC から各レールの電圧・電流と消費電力を算出する。"""
        if not (self.enable_pmic and self.has_vcgencmd):
            return None
        out = _run(['vcgencmd', 'pmic_read_adc'])
        if not out:
            return None

        currents = {}
        volts = {}
        for line in out.splitlines():
            m = _ADC_LINE.match(line)
            if not m:
                continue
            value = float(m.group('value'))
            if m.group('kind') == 'A':
                currents[m.group('rail')] = value
            else:
                volts[m.group('rail')] = value

        rails = {}
        total_w = 0.0
        for rail, current in currents.items():
            voltage = volts.get(rail)
            if voltage is None:
                continue
            watt = voltage * current
            total_w += watt
            rails[rail] = {
                'volt': round(voltage, 3),
                'amp': round(current, 3),
                'watt': round(watt, 3),
            }

        input_volt = volts.get('EXT5V')
        # PMIC は EXT5V の電流を出さないため、各レールの合計電力から換算する
        input_amp = total_w / input_volt if input_volt else None

        return {
            'total_w': round(total_w, 2),
            'input_volt': round(input_volt, 3) if input_volt else None,
            'input_amp': round(input_amp, 3) if input_amp else None,
            'core_volt': round(volts['VDD_CORE'], 3) if 'VDD_CORE' in volts else None,
            'rails': dict(sorted(rails.items(), key=lambda kv: -kv[1]['watt'])),
            'throttled': self._throttled(),
        }

    def _throttled(self):
        """get_throttled のビットを名前付きフラグに展開する。"""
        if not self.has_vcgencmd:
            return None
        out = _run(['vcgencmd', 'get_throttled'])
        if not out or '=' not in out:
            return None
        try:
            value = int(out.split('=')[1], 16)
        except ValueError:
            return None
        flags = {name: bool(value & (1 << bit)) for bit, name in _THROTTLE_BITS.items()}
        flags['raw'] = hex(value)
        return flags

    # --- AI HAT+ (Hailo-8) ---
    def _detect_hailo(self):
        """起動時に一度だけ AI HAT+ の存在とドライバ状況を調べる。"""
        if not self.enable_hailo:
            return {'enabled': False}

        lspci = _run(['lspci']) or ''
        device_line = next(
            (line for line in lspci.splitlines() if 'Hailo' in line), None)
        driver_ready = os.path.exists('/dev/hailo0')
        sysfs = _hailo_pci_sysfs()

        info = {
            'enabled': True,
            'detected': device_line is not None,
            'pci': device_line,
            'driver_ready': driver_ready,
            'cli_available': self.hailortcli is not None,
            'driver_version': _read_text('/sys/module/hailo_pci/version'),
            'pci_address': os.path.basename(sysfs) if sysfs else None,
            'device_node': '/dev/hailo0' if driver_ready else None,
            'link_speed': _read_text(f'{sysfs}/current_link_speed' if sysfs else None),
            'link_width': _read_text(f'{sysfs}/current_link_width' if sysfs else None),
            'max_link_speed': _read_text(f'{sysfs}/max_link_speed' if sysfs else None),
            'max_link_width': _read_text(f'{sysfs}/max_link_width' if sysfs else None),
        }
        if not device_line:
            info['note'] = 'Hailo-8 が PCIe 上に見つかりません'
        elif not driver_ready:
            info['note'] = 'PCIeでHailo-8を検出。hailo_pci ドライバ未導入のため詳細情報は取得不可'
        elif not info['cli_available']:
            info['note'] = 'ドライバ動作中。hailortcli 未導入のためファームウェア情報は取得不可'
        else:
            info.update(self._hailo_identify())
            # 電力測定はボードに DVM 非搭載のため不可
            info['note'] = 'Hailo-8 は電力測定・NPU使用率の取得に非対応（温度のみ取得可）'
        return info

    def _hailo_identify(self):
        out = _run([self.hailortcli, 'fw-control', 'identify'], timeout=10.0)
        if not out:
            return {}
        fields = {}
        for line in out.splitlines():
            if ':' not in line:
                continue
            key, _, value = line.partition(':')
            key = key.strip().lower().replace(' ', '_')
            if key in ('serial_number', 'part_number', 'product_name',
                       'firmware_version', 'device_architecture'):
                fields[key] = value.strip()
        return fields

    def _hailo(self):
        info = dict(self._hailo_static)
        if info.get('driver_ready'):
            info['temperature_c'] = _hailo_temperature()
        return info

    def _publish_cb(self):
        payload = {
            'stamp': round(self.get_clock().now().nanoseconds * 1e-9, 3),
            'uptime_s': int(psutil.boot_time() and
                            (self.get_clock().now().nanoseconds * 1e-9 - psutil.boot_time())),
            'cpu': self._cpu(),
            'memory': self._memory(),
            'power': self._power(),
            'hailo': self._hailo(),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitorNode()
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
