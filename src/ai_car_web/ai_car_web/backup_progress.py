"""バックアップ中のファイル（作っている・読み出している）を見つけて、進み具合を記録する。

scan_dir() は標準ライブラリだけで動く。Jetson にはこのファイルをそのまま
`python3 - /home/jetson/jetson-backups` で送って、同じ調べ方をする。
"""
import json
import os
import sys
import threading
import time

ACTIVE_WRITE_S = 90
STALL_S = 60
KEEP_ENDED_S = 15 * 60
KEEP_ENDED_PER_HOST = 1
MAX_SAMPLES = 1200


def _kind(comm, writing):
    if writing:
        return 'create'
    if comm.startswith('sha256'):
        return 'verify'
    if comm in ('tar', 'zstd', 'unzstd'):
        return 'restore'
    return 'copy'


def _manifests(d, names):
    done = {}
    for n in names:
        if not n.endswith('.manifest.json'):
            continue
        try:
            with open(os.path.join(d, n), encoding='utf-8') as f:
                done[n[:-len('.manifest.json')] + '.tar.zst'] = int(json.load(f).get('tar_bytes', 0))
        except (OSError, ValueError, TypeError):
            continue
    return done


def _open_files(d, proc):
    """d の中の .tar.zst を開いているプロセス（読めるものだけ）。"""
    found = {}
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return found
    for pid in pids:
        fd_dir = os.path.join(proc, pid, 'fd')
        try:
            fds = os.listdir(fd_dir)
        except OSError:
            continue
        for fd in fds:
            try:
                path = os.readlink(os.path.join(fd_dir, fd))
            except OSError:
                continue
            if os.path.dirname(path) != d or not path.endswith('.tar.zst'):
                continue
            try:
                with open(os.path.join(proc, pid, 'fdinfo', fd), encoding='ascii') as f:
                    info = dict(line.split(':', 1) for line in f if ':' in line)
                with open(os.path.join(proc, pid, 'comm'), encoding='utf-8') as f:
                    comm = f.read().strip()
                pos = int(info['pos'])
                writing = int(info['flags'], 8) & 3 != 0
            except (OSError, KeyError, ValueError):
                continue
            found[(int(pid), os.path.basename(path))] = (comm, pos, writing)
    return found


def scan_dir(d, proc='/proc', now=None):
    now = time.time() if now is None else now
    d = os.path.realpath(d)
    try:
        names = os.listdir(d)
    except OSError:
        return {'dir': d, 'now': now, 'jobs': [], 'complete': {}}
    complete = _manifests(d, names)
    sizes = [v for _, v in sorted(complete.items()) if v > 0]
    est = sizes[-1] if sizes else 0
    jobs = []
    writers = set()
    for (pid, name), (comm, pos, writing) in sorted(_open_files(d, proc).items()):
        try:
            size = os.stat(os.path.join(d, name)).st_size
        except OSError:
            continue
        if writing:
            if name in writers:
                continue
            writers.add(name)
            jobs.append({'kind': 'create', 'file': name, 'pid': None, 'comm': comm,
                         'bytes': size, 'total': est, 'estimated': True})
        else:
            jobs.append({'kind': _kind(comm, False), 'file': name, 'pid': pid, 'comm': comm,
                         'bytes': pos, 'total': size, 'estimated': False})
    for name in names:
        if not name.endswith('.tar.zst') or name in complete or name in writers:
            continue
        try:
            st = os.stat(os.path.join(d, name))
        except OSError:
            continue
        if now - st.st_mtime < ACTIVE_WRITE_S:
            jobs.append({'kind': 'create', 'file': name, 'pid': None, 'comm': '',
                         'bytes': st.st_size, 'total': est, 'estimated': True})
    return {'dir': d, 'now': now, 'jobs': jobs, 'complete': complete}


def _speed(samples, span_s=30.0):
    if len(samples) < 2:
        return 0.0
    t1, b1 = samples[-1]
    for t0, b0 in reversed(samples[:-1]):
        if t1 - t0 >= span_s:
            break
    return max(0.0, (b1 - b0) / (t1 - t0)) if t1 > t0 else 0.0


class ProgressTracker:
    """scan_dir() の結果を機械ごとに受け取り、仕事ごとの (経過秒, バイト) をためる。"""

    def __init__(self, interval_s=5.0):
        self.interval_s = interval_s
        self.jobs = {}
        self.lock = threading.Lock()

    @staticmethod
    def _key(host, job):
        pid = '' if job['kind'] == 'create' else job['pid']
        return f"{host}:{job['kind']}:{job['file']}:{pid}"

    def update(self, host, scan, now=None):
        now = time.time() if now is None else now
        with self.lock:
            seen = set()
            for j in scan.get('jobs', []):
                key = self._key(host, j)
                seen.add(key)
                rec = self.jobs.get(key)
                if rec is None or rec['state'] != 'run':
                    rec = {'host': host, 'kind': j['kind'], 'file': j['file'], 'comm': j['comm'],
                           'start': now, 'moved': now, 'samples': [], 'state': 'run'}
                    self.jobs[key] = rec
                if rec['samples'] and rec['samples'][-1][1] != j['bytes']:
                    rec['moved'] = now
                rec.update(bytes=j['bytes'], total=j['total'], estimated=j['estimated'], last=now)
                rec['samples'].append((round(now - rec['start'], 1), j['bytes']))
                if len(rec['samples']) > MAX_SAMPLES:
                    rec['samples'] = rec['samples'][::2]
            complete = scan.get('complete', {})
            for key, rec in self.jobs.items():
                if rec['host'] != host or key in seen:
                    continue
                if rec['state'] == 'run':
                    self._end(rec, complete, now)
                elif rec['state'] == 'stopped' and rec['kind'] == 'create' and rec['file'] in complete:
                    # root の sha256 照合はプロセスが見えないので、manifest があとからできることがある
                    rec['state'] = 'done'
                    rec['bytes'] = rec['total'] = complete[rec['file']]
                    rec['estimated'] = False
            self._prune(now)

    def lost(self, host, now=None):
        """host を調べられなかった（SSH 不通など）。動いていた仕事は「見えない」にする。"""
        now = time.time() if now is None else now
        with self.lock:
            for rec in self.jobs.values():
                if rec['host'] == host and rec['state'] == 'run':
                    rec['state'] = 'lost'
                    rec['ended'] = rec['last'] = now
            self._prune(now)

    def _prune(self, now):
        """終わった仕事は、機械ごとに一番新しい KEEP_ENDED_PER_HOST 件を KEEP_ENDED_S 秒だけ残す。"""
        ended = sorted((k for k, r in self.jobs.items() if r['state'] != 'run'),
                       key=lambda k: (self.jobs[k].get('ended', self.jobs[k]['last']), self.jobs[k]['start']),
                       reverse=True)
        kept = {}
        for key in ended:
            r = self.jobs[key]
            kept[r['host']] = kept.get(r['host'], 0) + 1
            if kept[r['host']] > KEEP_ENDED_PER_HOST or now - r['last'] > KEEP_ENDED_S:
                del self.jobs[key]

    def _end(self, rec, complete, now):
        rec['ended'] = now
        if rec['kind'] == 'create':
            if rec['file'] in complete:
                rec['state'] = 'done'
                rec['bytes'] = rec['total'] = complete[rec['file']]
                rec['estimated'] = False
            else:
                rec['state'] = 'stopped'
            return
        left = rec['total'] - rec['bytes']
        margin = _speed(rec['samples']) * self.interval_s * 2 + 1e6
        if left <= margin:
            rec['state'] = 'done'
            rec['bytes'] = rec['total']
        else:
            rec['state'] = 'stopped'

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        out = []
        with self.lock:
            for rec in self.jobs.values():
                state = rec['state']
                if state == 'run' and now - rec['moved'] > STALL_S:
                    state = 'stall'
                speed = _speed(rec['samples']) if rec['state'] == 'run' else 0.0
                total = rec['total']
                pct = min(100.0, 100.0 * rec['bytes'] / total) if total else None
                eta = (total - rec['bytes']) / speed if state == 'run' and speed > 0 and total else None
                step = max(1, len(rec['samples']) // 300)
                out.append({
                    'host': rec['host'], 'kind': rec['kind'], 'file': rec['file'], 'comm': rec['comm'],
                    'state': state, 'bytes': rec['bytes'], 'total': total, 'estimated': rec['estimated'],
                    'pct': None if pct is None else round(pct, 1), 'speed_bps': round(speed),
                    'eta_s': None if eta is None else round(max(0.0, eta)),
                    'start': rec['start'], 'elapsed_s': round(rec.get('ended', now) - rec['start']),
                    'samples': rec['samples'][::step] + ([] if step == 1 else rec['samples'][-1:]),
                })
        out.sort(key=lambda r: (r['state'] not in ('run', 'stall'), -r['start']))
        return {'now': now, 'active': any(r['state'] in ('run', 'stall') for r in out), 'jobs': out}


if __name__ == '__main__':
    print(json.dumps(scan_dir(sys.argv[1])))
