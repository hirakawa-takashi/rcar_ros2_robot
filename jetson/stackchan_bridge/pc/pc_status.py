"""パソコン（WSL）の状態を、Jetson の jetson_status.py と同じ形で AI-CAR の POST /api/mypc/status へ送る。

CPU・メモリ・ページファイル（スワップの欄）・C: ドライブ・起動時刻は Windows の値（PowerShell を 1 つ動かしたまま 1 秒ごとに読む）、
GPU は nvidia-smi、声の AI（whisper-server・Ollama）は WSL の中を見る。標準ライブラリだけで動く（pc-status.service）。
CPU の温度は LibreHardwareMonitor（管理者で動かしたまま）の WMI の「CPU Package」。更新の数は 1 時間ごと
（Ubuntu は apt、Windows は Windows Update）。返事で頼まれたら、Ubuntu（WSL）の再起動・止める（Windows の WMI から wsl.exe）と、
Ubuntu の更新（pc-upgrade.service）をする。Windows Update の入れ込みは管理者がいるのでしない。
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
from datetime import datetime

NVIDIA_SMI = '/usr/lib/wsl/lib/nvidia-smi'
POWERSHELL = '/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe'
WIN_LOOP = (
    "$ProgressPreference='SilentlyContinue';"
    "while($true){"
    "$c=(Get-Counter '\\Processor Information(_Total)\\% Processor Utility').CounterSamples.CookedValue;"
    "$o=Get-CimInstance Win32_OperatingSystem;"
    "$d=Get-CimInstance Win32_LogicalDisk -Filter \"DeviceID='C:'\";"
    "$p=@(Get-CimInstance Win32_PageFileUsage|Measure-Object -Property AllocatedBaseSize,CurrentUsage -Sum);"
    "$t=$null;foreach($ns in 'root/LibreHardwareMonitor','root/OpenHardwareMonitor'){try{"
    "$t=(Get-CimInstance -Namespace $ns -ClassName Sensor -Filter \"SensorType='Temperature' AND Name='CPU Package'\" "
    "-ErrorAction Stop|Select-Object -First 1).Value;if($t){break}}catch{}};"
    "$k='HKLM:\\SOFTWARE\\Microsoft\\Windows\\CurrentVersion';"
    "$rb=(Test-Path \"$k\\WindowsUpdate\\Auto Update\\RebootRequired\") -or "
    "(Test-Path \"$k\\Component Based Servicing\\RebootPending\");"
    "[pscustomobject]@{cpu=$c;mem_kb=$o.TotalVisibleMemorySize;free_kb=$o.FreePhysicalMemory;"
    "pf_mb=$p[0].Sum;pf_used_mb=$p[1].Sum;boot=$o.LastBootUpTime.ToString('o');"
    "os=$o.Caption;disk=$d.Size;disk_free=$d.FreeSpace;cpu_temp=$t;reboot=$rb}|ConvertTo-Json -Compress;"
    "[Console]::Out.Flush()}"
)
WIN_UPDATES = (
    "$u=@((New-Object -ComObject Microsoft.Update.Session).CreateUpdateSearcher()"
    ".Search('IsInstalled=0 and IsHidden=0').Updates);"
    "[pscustomobject]@{n=$u.Count;sec=@($u|Where-Object{$_.MsrcSeverity}).Count}|ConvertTo-Json -Compress"
)
WSL_RESTART = 'wsl.exe --terminate Ubuntu; Start-Sleep 5; wsl.exe -d Ubuntu --exec /bin/sleep infinity'
UPGRADE_UNIT = 'pc-upgrade.service'
UPGRADE_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'start', '--no-block', UPGRADE_UNIT]
IP_SKIP = ('lo', 'docker', 'br-', 'veth')


class Windows:
    """PowerShell を動かしたままにして、1 秒ごとに出る 1 行の JSON の最後を覚える（止まったら 10 秒後に起動し直す）。"""

    def __init__(self):
        self.latest = {}
        self.stamp = 0.0

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                proc = subprocess.Popen([POWERSHELL, '-NoProfile', '-Command', WIN_LOOP],
                                        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                        stdin=subprocess.DEVNULL, text=True)
                for line in proc.stdout:
                    try:
                        self.latest = json.loads(line)
                        self.stamp = time.monotonic()
                    except ValueError:
                        pass
                proc.wait()
            except OSError as e:
                print(f'PowerShell を動かせません: {e}', file=sys.stderr, flush=True)
            time.sleep(10)

    def get(self):
        return self.latest if time.monotonic() - self.stamp < 10 else {}


class Updates:
    """更新の数を 1 時間ごと（と Ubuntu の更新のあと）に数える。Windows Update を調べるのに約 15 秒かかる。"""

    def __init__(self):
        self.latest = {}
        self.wake = threading.Event()

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            latest = {}
            sim = run(['apt-get', '-s', '-o', 'Debug::NoLocking=1', 'upgrade'], timeout=120)
            inst = [line for line in sim.splitlines() if line.startswith('Inst ')]
            if sim:
                latest.update(ubuntu_updates=len(inst), ubuntu_security=sum('-security' in line for line in inst))
            try:
                w = json.loads(run([POWERSHELL, '-NoProfile', '-Command', WIN_UPDATES], timeout=300))
                latest.update(win_updates_pending=int(w['n']), win_security_pending=int(w['sec']))
            except (ValueError, KeyError, TypeError):
                pass
            self.latest = latest
            self.wake.wait(3600)
            self.wake.clear()


def can_sudo(cmd):
    """pc-upgrade.sudoers でパスワードなしに cmd を実行できるか。"""
    out = run(['sudo', '-n', '-l'])
    want = ' '.join(cmd[2:])
    return any('NOPASSWD:' in line and line.rstrip().endswith(want) for line in out.splitlines())


def upgrade_state():
    """ボタンの更新（pc-upgrade.service）が動いているかと、前回の結果（一度も動いていなければ空）。"""
    out = run(['systemctl', 'show', UPGRADE_UNIT, '-p', 'ActiveState', '-p', 'Result',
               '-p', 'ExecMainStartTimestampMonotonic'], timeout=3)
    props = dict(line.split('=', 1) for line in out.splitlines() if '=' in line)
    running = props.get('ActiveState') in ('activating', 'active', 'deactivating')
    started = props.get('ExecMainStartTimestampMonotonic', '0') not in ('', '0')
    return {'upgrade_running': running,
            'upgrade_result': props.get('Result', '') if started and not running else ''}


def windows_detached(command):
    """Windows の WMI に PowerShell を起こさせる（WSL を止めても消えない）。"""
    line = f'powershell.exe -NoProfile -WindowStyle Hidden -Command "{command}"'
    run([POWERSHELL, '-NoProfile', '-Command',
         f"Invoke-CimMethod Win32_Process -MethodName Create -Arguments @{{CommandLine='{line}'}}"], timeout=30)


def reply_flag(reply, key):
    try:
        return json.loads(reply).get(key) is True
    except (ValueError, AttributeError):
        return False


def http_ok(url, timeout=1.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as res:
            return res.status == 200, res.read()
    except (urllib.error.URLError, OSError):
        return False, b''


def run(cmd, timeout=5):
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return ''


def gpu():
    out = run([NVIDIA_SMI, '--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,power.draw,'
               'clocks.gr,memory.used,memory.total', '--format=csv,noheader,nounits']).strip()
    parts = [p.strip() for p in out.splitlines()[0].split(',')] if out else []
    if len(parts) != 8:
        return {}

    def num(s):
        try:
            return float(s)
        except ValueError:
            return None
    return {'gpu_name': parts[0].removeprefix('NVIDIA ').removeprefix('GeForce '), 'gpu_driver': parts[1],
            'temp': num(parts[2]), 'load': num(parts[3]), 'power': num(parts[4]), 'mhz': num(parts[5])}


def services():
    active = {k: run(['systemctl', 'is-active', unit]).strip() == 'active'
              for k, unit in (('whisper', 'whisper-server'), ('ollama', 'ollama'))}
    active['whisper'] = active['whisper'] and http_ok('http://127.0.0.1:8178/')[0]
    return active


def stt_engine():
    m = re.search(r'\s(?:-m|--model)\s+(\S+)', run(['systemctl', 'show', '-p', 'ExecStart', '--value',
                                                    'whisper-server']))
    if not m:
        return ''
    return 'whisper.cpp ' + os.path.basename(m.group(1)).removeprefix('ggml-').removesuffix('.bin') + '（GPU）'


def ollama_models():
    empty = {'llm_models': [], 'llm_size_mb': None, 'llm_vram_mb': None}
    ok, body = http_ok('http://127.0.0.1:11434/api/ps')
    if not ok:
        return empty
    try:
        models = json.loads(body).get('models', [])[:4]
        return {'llm_models': [str(m.get('name', '')) for m in models],
                'llm_size_mb': round(sum(int(m.get('size', 0)) for m in models) / 2**20),
                'llm_vram_mb': round(sum(int(m.get('size_vram', 0)) for m in models) / 2**20)}
    except (ValueError, TypeError, AttributeError):
        return empty


def ip_addresses():
    ips = []
    for line in run(['ip', '-4', '-o', 'addr', 'show'], timeout=3).splitlines():
        parts = line.split()
        if len(parts) >= 4 and parts[2] == 'inet' and not parts[1].startswith(IP_SKIP):
            ips.append({'iface': parts[1], 'addr': parts[3].split('/')[0]})
    return ips


def wsl_os():
    m = re.search(r'^PRETTY_NAME="?([^"\n]+)', open('/etc/os-release', encoding='utf-8').read(), re.M)
    return m.group(1) if m else ''


def collect(win, slow, updates):
    w = win.get()
    u = updates.latest
    g = gpu()
    mb = lambda kb: round(kb / 1024) if isinstance(kb, (int, float)) else None  # noqa: E731
    status = {
        'hostname': os.uname().nodename,
        'os': f"{w.get('os', 'Windows').removeprefix('Microsoft ')}・WSL {slow['os']}",
        'gpu_name': g.get('gpu_name', ''),
        'gpu_driver': g.get('gpu_driver', ''),
        'cpu_percent': round(min(100.0, w['cpu']), 1) if isinstance(w.get('cpu'), (int, float)) else None,
        'gpu_percent': g.get('load'),
        'gpu_freq_mhz': g.get('mhz'),
        'temperatures_c': {k: v for k, v in (('cpu', w.get('cpu_temp')), ('gpu', g.get('temp')))
                           if isinstance(v, (int, float)) and v > 0},
        'power_w': g.get('power'),
        'services': services(),
        'stt_engine': slow['stt'],
        'ips': slow['ips'],
        'can_reboot': slow['can_wsl'],
        'can_upgrade': slow['can_upgrade'],
        'reboot_required': os.path.exists('/var/run/reboot-required'),
        'win_reboot_required': bool(w['reboot']) if 'reboot' in w else None,
    }
    if 'ubuntu_updates' in u:
        status.update(updates_pending=u['ubuntu_updates'] + u.get('win_updates_pending', 0),
                      security_pending=u['ubuntu_security'] + u.get('win_security_pending', 0),
                      ubuntu_updates_pending=u['ubuntu_updates'], ubuntu_security_pending=u['ubuntu_security'],
                      win_updates_pending=u.get('win_updates_pending'),
                      win_security_pending=u.get('win_security_pending'))
    status.update(upgrade_state())
    if w.get('mem_kb') and w.get('free_kb') is not None:
        status.update(mem_total_mb=mb(w['mem_kb']), mem_used_mb=mb(w['mem_kb'] - w['free_kb']),
                      mem_available_mb=mb(w['free_kb']))
    if isinstance(w.get('pf_mb'), (int, float)):
        status.update(swap_total_mb=w['pf_mb'], swap_used_mb=w.get('pf_used_mb'))
    if w.get('disk'):
        status.update(disk_total_gb=round(w['disk'] / 1e9, 1),
                      disk_used_gb=round((w['disk'] - w.get('disk_free', 0)) / 1e9, 1))
    try:
        status['uptime_s'] = round(time.time() - datetime.fromisoformat(w['boot']).timestamp())
    except (KeyError, TypeError, ValueError):
        pass
    status.update(ollama_models())
    return status


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--car-url', default=os.environ.get('AI_CAR_URL', 'http://100.70.35.31:8080'))
    parser.add_argument('--interval', type=float, default=1.0, help='送る間隔 [秒]')
    parser.add_argument('--once', action='store_true', help='1 回だけ集めて表示する（送らない）')
    args = parser.parse_args()

    token = os.environ.get('AI_CAR_API_TOKEN', '')
    url = args.car_url.rstrip('/') + '/api/mypc/status'
    win = Windows()
    win.start()
    updates = Updates()
    updates.start()
    slow = {'os': wsl_os(), 'stt': stt_engine(), 'ips': ip_addresses(), 'stamp': time.monotonic(),
            'can_wsl': os.access(POWERSHELL, os.X_OK), 'can_upgrade': can_sudo(UPGRADE_CMD)}
    if args.once:
        time.sleep(25)
        print(json.dumps(collect(win, slow, updates), ensure_ascii=False, indent=1))
        return
    print(f'パソコンの状態を送ります → {url}', flush=True)
    failing = False
    upgrading = False
    while True:
        time.sleep(args.interval)
        if time.monotonic() - slow['stamp'] > 60:
            slow.update(stt=stt_engine(), ips=ip_addresses(), can_upgrade=can_sudo(UPGRADE_CMD),
                        stamp=time.monotonic())
        status = collect(win, slow, updates)
        if upgrading and not status['upgrade_running']:
            updates.wake.set()
        upgrading = status['upgrade_running']
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
                print('ダッシュボードから更新を頼まれたので pc-upgrade.service を始めます', flush=True)
                subprocess.run(UPGRADE_CMD, timeout=30, check=False)
            if reply_flag(reply, 'reboot'):
                print('ダッシュボードから再起動を頼まれたので Ubuntu を再起動します', flush=True)
                windows_detached(WSL_RESTART)
        except (urllib.error.URLError, OSError, subprocess.TimeoutExpired) as e:
            if not failing:
                print(f'送れません: {e}', file=sys.stderr, flush=True)
            failing = True


if __name__ == '__main__':
    main()
