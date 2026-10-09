"""パソコン（WSL）の状態を、Jetson の jetson_status.py と同じ形で AI-CAR の POST /api/mypc/status へ送る。

CPU・メモリ・ページファイル（スワップの欄）・C: ドライブ・起動時刻は Windows の値（PowerShell を 1 つ動かしたまま 1 秒ごとに読む）、
GPU は nvidia-smi、声の AI（whisper-server・Ollama）は WSL の中を見る。標準ライブラリだけで動く（pc-status.service）。
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
    "[pscustomobject]@{cpu=$c;mem_kb=$o.TotalVisibleMemorySize;free_kb=$o.FreePhysicalMemory;"
    "pf_mb=$p[0].Sum;pf_used_mb=$p[1].Sum;boot=$o.LastBootUpTime.ToString('o');"
    "os=$o.Caption;disk=$d.Size;disk_free=$d.FreeSpace}|ConvertTo-Json -Compress;"
    "[Console]::Out.Flush()}"
)
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


def collect(win, slow):
    w = win.get()
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
        'temperatures_c': {'gpu': g['temp']} if g.get('temp') is not None else {},
        'power_w': g.get('power'),
        'services': services(),
        'stt_engine': slow['stt'],
        'ips': slow['ips'],
    }
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
    slow = {'os': wsl_os(), 'stt': stt_engine(), 'ips': ip_addresses(), 'stamp': time.monotonic()}
    if args.once:
        time.sleep(5)
        print(json.dumps(collect(win, slow), ensure_ascii=False, indent=1))
        return
    print(f'パソコンの状態を送ります → {url}', flush=True)
    failing = False
    while True:
        time.sleep(args.interval)
        if time.monotonic() - slow['stamp'] > 60:
            slow.update(stt=stt_engine(), ips=ip_addresses(), stamp=time.monotonic())
        req = urllib.request.Request(
            url, data=json.dumps(collect(win, slow)).encode('utf-8'), method='POST',
            headers={'Content-Type': 'application/json', 'X-API-Token': token})
        try:
            with urllib.request.urlopen(req, timeout=3.0) as res:
                res.read()
            if failing:
                print('送れるようになりました', flush=True)
            failing = False
        except (urllib.error.URLError, OSError) as e:
            if not failing:
                print(f'送れません: {e}', file=sys.stderr, flush=True)
            failing = True


if __name__ == '__main__':
    main()
