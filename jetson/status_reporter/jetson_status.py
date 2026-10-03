"""Jetson の状態（温度・使用率・メモリ・電力・AI の部品・更新）を AI-CAR へ送る。

1 秒ごとに /proc・/sys と、声の AI（whisper-server・Ollama・Kokoro）の HTTP を見て、
POST /api/jetson/status へ送る。セキュリティ更新の数（apt-check）は重いので 1 時間に 1 回だけ調べる。
標準ライブラリだけで動く（jetson-status.service を参照）。
"""

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request

GPU_DIR = '/sys/devices/platform/bus@0/17000000.gpu'
THERMAL_NAMES = {'cpu-thermal': 'cpu', 'gpu-thermal': 'gpu', 'tj-thermal': 'tj'}
IP_SKIP = ('lo', 'docker', 'br-', 'veth', 'l4tbr')
REBOOT_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'reboot']
UPGRADE_UNIT = 'jetson-upgrade.service'
UPGRADE_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'start', '--no-block', UPGRADE_UNIT]
UPGRADE_PROGRESS = '/run/jetson-upgrade.progress'
HOLD_PREFIXES = ('nvidia-', 'cuda-', 'libcudnn', 'libnvinfer', 'tensorrt')
SERVICES = {'whisper': 'whisper-server', 'ollama': 'ollama', 'nanoowl': 'jetson-owl'}


def read(path):
    try:
        with open(path, encoding='utf-8') as f:
            return f.read().strip()
    except OSError:
        return None


def read_number(path):
    text = read(path)
    try:
        return float(text) if text is not None else None
    except ValueError:
        return None


def cpu_times():
    fields = [float(v) for v in read('/proc/stat').splitlines()[0].split()[1:]]
    idle = fields[3] + fields[4]
    return sum(fields), idle


def cpu_freq_mhz():
    freqs = []
    for name in os.listdir('/sys/devices/system/cpu'):
        if re.fullmatch(r'cpu\d+', name):
            khz = read_number(f'/sys/devices/system/cpu/{name}/cpufreq/scaling_cur_freq')
            if khz:
                freqs.append(khz / 1000.0)
    return round(sum(freqs) / len(freqs)) if freqs else None


def temperatures():
    temps = {}
    base = '/sys/class/thermal'
    for zone in sorted(os.listdir(base)):
        key = THERMAL_NAMES.get(read(f'{base}/{zone}/type') or '')
        milli = read_number(f'{base}/{zone}/temp') if key else None
        if key and milli is not None:
            temps[key] = round(milli / 1000.0, 1)
    return temps


def hwmon(name):
    base = '/sys/class/hwmon'
    for h in os.listdir(base):
        if read(f'{base}/{h}/name') == name:
            return f'{base}/{h}'
    return None


def power():
    """INA3221 の VDD_IN（ボード全体の入力）の電圧と電力。"""
    h = hwmon('ina3221')
    if h is None:
        return None, None
    for i in range(1, 4):
        if read(f'{h}/in{i}_label') == 'VDD_IN':
            mv = read_number(f'{h}/in{i}_input')
            ma = read_number(f'{h}/curr{i}_input')
            if mv is None or ma is None:
                return None, None
            return round(mv / 1000.0, 2), round(mv * ma / 1e6, 2)
    return None, None


def fan():
    pwm_dir, tach_dir = hwmon('pwmfan'), hwmon('pwm_tach')
    pwm = read_number(f'{pwm_dir}/pwm1') if pwm_dir else None
    rpm = read_number(f'{tach_dir}/rpm') if tach_dir else None
    return (round(pwm / 255.0 * 100.0) if pwm is not None else None,
            int(rpm) if rpm is not None else None)


def memory():
    info = {}
    for line in read('/proc/meminfo').splitlines():
        key, value = line.split(':', 1)
        info[key] = float(value.split()[0]) / 1024.0
    return {
        'mem_total_mb': round(info['MemTotal']),
        'mem_used_mb': round(info['MemTotal'] - info['MemAvailable']),
        'mem_available_mb': round(info['MemAvailable']),
        'swap_total_mb': round(info['SwapTotal']),
        'swap_used_mb': round(info['SwapTotal'] - info['SwapFree']),
    }


def l4t_version():
    m = re.search(r'R(\d+) \(release\), REVISION: ([\d.]+)', read('/etc/nv_tegra_release') or '')
    return f'{m.group(1)}.{m.group(2)}' if m else ''


def os_name():
    m = re.search(r'^PRETTY_NAME="?([^"\n]+)', read('/etc/os-release') or '', re.M)
    return m.group(1) if m else ''


def http_ok(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as res:
            return res.status == 200, res.read()
    except (urllib.error.URLError, OSError):
        return False, b''


def services():
    states = {}
    for key, unit in SERVICES.items():
        result = subprocess.run(['systemctl', 'is-active', unit], capture_output=True,
                                text=True, check=False)
        states[key] = result.stdout.strip() == 'active'
    states['whisper'] = states['whisper'] and http_ok('http://127.0.0.1:8178/')[0]
    states['kokoro'] = http_ok('http://127.0.0.1:8880/health')[0]
    return states


def ollama_models():
    """読み込み中のモデルの名前と、大きさ・GPU に載っている量の合計 [MB]（Ollama の /api/ps）。"""
    empty = {'llm_models': [], 'llm_size_mb': None, 'llm_vram_mb': None}
    ok, body = http_ok('http://127.0.0.1:11434/api/ps')
    if not ok:
        return empty
    try:
        models = json.loads(body).get('models', [])[:4]
        return {
            'llm_models': [str(m.get('name', '')) for m in models],
            'llm_size_mb': round(sum(int(m.get('size', 0)) for m in models) / 2**20),
            'llm_vram_mb': round(sum(int(m.get('size_vram', 0)) for m in models) / 2**20),
        }
    except (ValueError, TypeError, AttributeError):
        return empty


class SlowFacts:
    """apt-check と nvpmodel は遅い（apt-check は約 10 秒）ので、別スレッドで 1 時間ごと（と更新のあと）に調べる。"""

    def __init__(self, interval):
        self.interval = interval
        self.lock = threading.Lock()
        self.facts = {}
        self.wake = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            self.refresh()
            self.wake.wait(self.interval)
            self.wake.clear()

    def refresh(self):
        facts = {'power_mode': self._power_mode(), 'can_reboot': can_sudo(REBOOT_CMD),
                 'can_upgrade': can_sudo(UPGRADE_CMD)}
        updates, facts['security_pending'] = self._apt_check()
        held = self._held_count()
        facts['updates_held'] = held
        facts['updates_pending'] = (updates - held if updates is not None and held is not None
                                    else updates)
        with self.lock:
            self.facts = facts

    @staticmethod
    def _power_mode():
        try:
            out = subprocess.run(['nvpmodel', '-q'], capture_output=True, text=True,
                                 timeout=10, check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            return ''
        m = re.search(r'NV Power Mode: (\S+)', out)
        return m.group(1) if m else ''

    @staticmethod
    def _apt_check():
        try:
            out = subprocess.run(['nice', '-n', '19', '/usr/lib/update-notifier/apt-check'],
                                 capture_output=True, text=True, timeout=120,
                                 check=False).stderr
        except (OSError, subprocess.TimeoutExpired):
            return None, None
        m = re.search(r'(\d+);(\d+)', out)
        return (int(m.group(1)), int(m.group(2))) if m else (None, None)

    @staticmethod
    def _held_count():
        """更新できる部品のうち、NVIDIA の部品（JetPack。自動でも手動のボタンでも変えない）の数。"""
        try:
            out = subprocess.run(['nice', '-n', '19', 'apt', 'list', '--upgradable'],
                                 capture_output=True, text=True, timeout=120,
                                 check=False).stdout
        except (OSError, subprocess.TimeoutExpired):
            return None
        names = [line.split('/', 1)[0] for line in out.splitlines() if '/' in line]
        return sum(name.startswith(HOLD_PREFIXES) for name in names)

    def get(self):
        with self.lock:
            return dict(self.facts)


def ip_addresses():
    """IPv4 アドレスを [{'iface': ..., 'addr': ...}] で返す（lo・Docker・USB の l4tbr0 は除く）。"""
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr', 'show'], capture_output=True,
                             text=True, timeout=3, check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return []
    ips = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == 'inet' and not parts[1].startswith(IP_SKIP):
            ips.append({'iface': parts[1], 'addr': parts[3].split('/')[0]})
    return ips


_ips_cache = {'stamp': 0.0, 'ips': []}


def ip_addresses_cached(max_age=30.0):
    now = time.monotonic()
    if not _ips_cache['stamp'] or now - _ips_cache['stamp'] >= max_age:
        _ips_cache['ips'] = ip_addresses()
        _ips_cache['stamp'] = now
    return _ips_cache['ips']


def can_sudo(cmd):
    """sudoers（jetson-reboot.sudoers・jetson-upgrade.sudoers）でパスワードなしに cmd を実行できるか。"""
    try:
        out = subprocess.run(['sudo', '-n', '-l'], capture_output=True, text=True,
                             timeout=5, check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    want = ' '.join(cmd[2:])
    return any('NOPASSWD:' in line and line.rstrip().endswith(want) for line in out.splitlines())


def last_upgrade():
    try:
        return round(os.path.getmtime('/var/lib/apt/periodic/unattended-upgrades-stamp'))
    except OSError:
        return None


def upgrade_progress(path):
    """更新スクリプトが書く「<0〜100> <prepare|download|install>」を読む。"""
    try:
        with open(path, encoding='utf-8') as f:
            percent, phase = f.read().split()
        return {'upgrade_percent': max(0, min(100, int(percent))), 'upgrade_phase': phase}
    except (OSError, ValueError):
        return {'upgrade_percent': None, 'upgrade_phase': ''}


def upgrade_state():
    """ボタンの更新（jetson-upgrade.service）が動いているかと、前回の結果（一度も動いていなければ空）。"""
    try:
        out = subprocess.run(['systemctl', 'show', UPGRADE_UNIT, '-p', 'ActiveState', '-p', 'Result',
                              '-p', 'ExecMainStartTimestampMonotonic'],
                             capture_output=True, text=True, timeout=3, check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return {'upgrade_running': None, 'upgrade_result': ''}
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    running = props.get('ActiveState') in ('activating', 'active', 'deactivating')
    started = props.get('ExecMainStartTimestampMonotonic', '0') not in ('', '0')
    state = {'upgrade_running': running,
             'upgrade_result': props.get('Result', '') if started and not running else ''}
    state.update(upgrade_progress(UPGRADE_PROGRESS) if running
                 else {'upgrade_percent': None, 'upgrade_phase': ''})
    return state


def collect(prev_cpu, slow):
    total, idle = cpu_times()
    d_total, d_idle = total - prev_cpu[0], idle - prev_cpu[1]
    gpu_load = read_number(f'{GPU_DIR}/load')
    gpu_hz = None
    devfreq = f'{GPU_DIR}/devfreq'
    if os.path.isdir(devfreq):
        for name in os.listdir(devfreq):
            gpu_hz = read_number(f'{devfreq}/{name}/cur_freq')
    volt, watt = power()
    fan_pct, fan_rpm = fan()
    disk = os.statvfs('/')
    uptime = float(read('/proc/uptime').split()[0])
    status = {
        'hostname': os.uname().nodename,
        'os': os_name(),
        'l4t': l4t_version(),
        'uptime_s': round(uptime),
        'cpu_percent': round(100.0 * (1.0 - d_idle / d_total), 1) if d_total > 0 else None,
        'cpu_freq_mhz': cpu_freq_mhz(),
        'gpu_percent': round(gpu_load / 10.0, 1) if gpu_load is not None else None,
        'gpu_freq_mhz': round(gpu_hz / 1e6) if gpu_hz else None,
        'temperatures_c': temperatures(),
        'input_volt': volt,
        'power_w': watt,
        'fan_percent': fan_pct,
        'fan_rpm': fan_rpm,
        'disk_total_gb': round(disk.f_blocks * disk.f_frsize / 1e9, 1),
        'disk_used_gb': round((disk.f_blocks - disk.f_bfree) * disk.f_frsize / 1e9, 1),
        'services': services(),
        'reboot_required': os.path.exists('/var/run/reboot-required'),
        'last_upgrade': last_upgrade(),
        'ips': ip_addresses_cached(),
    }
    status.update(memory())
    status.update(ollama_models())
    status.update(upgrade_state())
    status.update(slow.get())
    return status, (total, idle)


def reply_flag(reply, key):
    try:
        return json.loads(reply).get(key) is True
    except (ValueError, AttributeError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--car-url',
                        default=os.environ.get('AI_CAR_URL', 'http://100.70.35.31:8080'))
    parser.add_argument('--interval', type=float, default=1.0, help='送る間隔 [秒]')
    parser.add_argument('--slow-interval', type=float, default=3600.0,
                        help='セキュリティ更新の数を調べる間隔 [秒]')
    parser.add_argument('--once', action='store_true', help='1 回だけ集めて表示する（送らない）')
    args = parser.parse_args()

    token = os.environ.get('AI_CAR_API_TOKEN', '')
    url = args.car_url.rstrip('/') + '/api/jetson/status'
    slow = SlowFacts(args.slow_interval)
    prev_cpu = cpu_times()
    if args.once:
        slow.refresh()
        status, _ = collect(prev_cpu, slow)
        print(json.dumps(status, ensure_ascii=False, indent=1))
        return
    slow.start()
    print(f'Jetson の状態を送ります → {url}', flush=True)
    failing = False
    upgrading = False
    while True:
        time.sleep(args.interval)
        status, prev_cpu = collect(prev_cpu, slow)
        if upgrading and status['upgrade_running'] is False:
            slow.wake.set()
        upgrading = bool(status['upgrade_running'])
        req = urllib.request.Request(
            url, data=json.dumps(status).encode('utf-8'), method='POST',
            headers={'Content-Type': 'application/json', 'X-API-Token': token})
        try:
            with urllib.request.urlopen(req, timeout=3.0) as res:
                reply = res.read()
            if failing:
                print('送れるようになりました', flush=True)
            failing = False
            if reply_flag(reply, 'upgrade'):
                print('ダッシュボードから更新を頼まれたので jetson-upgrade.service を始めます', flush=True)
                subprocess.run(UPGRADE_CMD, timeout=30, check=False)
            if reply_flag(reply, 'reboot'):
                print('ダッシュボードから再起動を頼まれたので再起動します', flush=True)
                subprocess.run(REBOOT_CMD, timeout=30, check=False)
        except (urllib.error.URLError, OSError, subprocess.TimeoutExpired) as e:
            if not failing:
                print(f'送れません: {e}', file=sys.stderr, flush=True)
            failing = True


if __name__ == '__main__':
    main()
