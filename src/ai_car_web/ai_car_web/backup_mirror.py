"""バックアップを PC へミラーした記録（~/mirror-pi.log・~/mirror-jetson.log）を読み、
まだ PC にない一番新しいバックアップができたら scripts/ai-backup-mirror.sh を動かす（定時ではない）。"""
import os
import re
import subprocess
import time

LOG_RE = re.compile(r'^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}) (OK|NG) (.*)$')
RETRY_S = 10 * 60
KINDS = ('pi', 'jetson')


def read_log(path, tail_bytes=65536):
    """{'ok': {ファイル名: 照合した時刻}, 'last': 最後の OK/NG の行}。"""
    try:
        with open(path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - tail_bytes))
            text = f.read().decode('utf-8', 'replace')
    except OSError:
        return {'ok': {}, 'last': None}
    ok, last = {}, None
    for line in text.splitlines():
        m = LOG_RE.match(line.strip())
        if not m:
            continue
        when, res, rest = m.groups()
        parts = rest.split()
        if res == 'OK' and len(parts) == 3 and parts[0] == 'sha256':
            ok[parts[2]] = when
        last = {'time': when, 'ok': res == 'OK', 'text': rest}
    return {'ok': ok, 'last': last}


class AutoMirror:
    def __init__(self, script, home, enabled=True):
        self.script = script
        self.home = home
        self.enabled = bool(enabled and script and os.path.isfile(script))
        self.procs = {}
        self.tried = {}

    def log_path(self, kind):
        return os.path.join(self.home, f'mirror-{kind}.log')

    def running(self, kind):
        p = self.procs.get(kind)
        return p is not None and p.poll() is None

    def maybe_start(self, kind, complete, busy, now=None):
        """complete は scan_dir() の complete（manifest のある tar）。busy はその機械でバックアップ中。"""
        now = time.time() if now is None else now
        if not self.enabled or busy or not complete or self.running(kind):
            return None
        newest = max(complete)
        if newest in read_log(self.log_path(kind))['ok']:
            return None
        if now - self.tried.get(newest, float('-inf')) < RETRY_S:
            return None
        self.tried[newest] = now
        self.procs[kind] = subprocess.Popen(
            ['bash', self.script, kind, newest], stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        return newest

    def status(self):
        out = {'enabled': self.enabled}
        for kind in KINDS:
            log = read_log(self.log_path(kind))
            ok = log['ok']
            f = max(ok, key=ok.get) if ok else None
            out[kind] = {'running': self.running(kind), 'ok_file': f, 'ok_time': ok.get(f), 'last': log['last']}
        return out
