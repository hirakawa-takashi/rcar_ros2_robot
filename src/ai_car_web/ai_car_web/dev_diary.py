"""開発日記: git の main の記録（PR のマージ）と CHANGELOG の追加行から日付ごとの変更一覧を作る。

1 件ずつ SQLite の DB にためて、新しい版が入ったら増えた分だけ足す（再起動しても作り直さない）。
"""

import json
import os
import re
import sqlite3
import subprocess
import threading
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone

JST = timezone(timedelta(hours=9))
EMPTY_TREE = '4b825dc642cb6eb9a060e54bf8d69288fbee4904'
MERGE_RE = re.compile(r'^Merge pull request #(\d+) from ')
GITHUB_RE = re.compile(r'github\.com[/:]([^/\s]+)/([^/\s]+?)(?:\.git)?/?$')
TRAILER_RE = re.compile(r'^[\w-]+-by:', re.IGNORECASE)
WIRING_FILES = ('gpio_pins.yaml', 'motor_hat.yaml', 'HARDWARE_BOM.md')

SCHEMA = '1'   # _entry の中身を変えたら上げる（DB を作り直す）
LOG_FORMAT = '%H%x1f%P%x1f%cI%x1f%s%x1f%b%x1e'

_cache = {}
_lock = threading.Lock()


def _git(repo, *args):
    return subprocess.run(['git', '-C', repo, *args], capture_output=True, text=True,
                          timeout=10, check=True).stdout


def find_repo(start):
    """start を含む git リポジトリの最上位。見つからなければ None。"""
    try:
        return _git(start, 'rev-parse', '--show-toplevel').strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


def repo_url(remote):
    m = GITHUB_RE.search(remote.strip())
    return f'https://github.com/{m.group(1)}/{m.group(2)}' if m else ''


def classify(paths):
    """変更したファイルから分類（3D プリント / 配線 / ダッシュボード / ソフト / ドキュメント）。"""
    tags = []
    if any(p.startswith('hardware/3d/') for p in paths):
        tags.append('3D プリント')
    if any(os.path.basename(p) in WIRING_FILES for p in paths):
        tags.append('配線')
    if any('/static/' in p for p in paths):
        tags.append('ダッシュボード')
    if any(p.endswith('.py') or '/launch/' in p or p.endswith('dashboard.yaml') for p in paths):
        tags.append('ソフト')
    if not tags and any(p.endswith('.md') for p in paths):
        tags.append('ドキュメント')
    return tags


def changelog_lines(diff):
    """CHANGELOG.md の差分から、追加された箇条書きの本文を取り出す。"""
    return [line[3:].strip() for line in diff.splitlines()
            if line.startswith('+- ') and line[3:].strip()]


def pr_title(repo, parents, body):
    """マージの本文の 1 行目（PR の題名）。なければ PR の最初のコミットの件名。"""
    for line in body.splitlines():
        line = line.strip().lstrip('* ').strip()
        if line and not TRAILER_RE.match(line):
            return line
    if len(parents) > 1:
        subjects = _git(repo, 'log', '--reverse', '--format=%s',
                        f'{parents[0]}..{parents[1]}').splitlines()
        if subjects:
            return subjects[0]
    return ''


def _entry(repo, url, sha, parents, stamp, subject, body):
    base = parents[0] if parents else EMPTY_TREE
    paths = [p for p in _git(repo, 'diff', '--name-only', base, sha).splitlines() if p]
    when = datetime.fromisoformat(stamp).astimezone(JST)
    m = MERGE_RE.match(subject)
    entry = {
        'sha': sha[:7],
        'date': when.strftime('%Y-%m-%d'),
        'time': when.strftime('%H:%M'),
        'kind': 'pr' if m else 'commit',
        'number': int(m.group(1)) if m else None,
        'title': (pr_title(repo, parents, body) or subject) if m else subject,
        'changes': changelog_lines(_git(repo, 'diff', '-U0', base, sha, '--', 'CHANGELOG.md')),
        'prints': sorted(os.path.basename(p) for p in paths
                         if p.startswith('hardware/3d/') and p.endswith('.stl')),
        'tags': classify(paths),
        'files': len(paths),
    }
    if url:
        entry['url'] = (f'{url}/pull/{entry["number"]}' if m else f'{url}/commit/{sha}')
    return entry


def _log_records(repo):
    """main（最初の親をたどる）の記録を新しい順に (sha, parents, stamp, subject, body)。"""
    for record in _git(repo, 'log', '--first-parent', f'--format={LOG_FORMAT}').split('\x1e'):
        fields = record.strip('\n').split('\x1f')
        if len(fields) >= 5:
            yield fields[:5]


def _days(entries):
    days = {}
    for entry in entries:
        days.setdefault(entry['date'], []).append(entry)
    return [{'date': d, 'entries': days[d]} for d in sorted(days, reverse=True)]


def load_dev_diary(repo):
    """main の記録を全部、新しい日付順にまとめて返す（DB を使わない。手元の表示確認用）。"""
    if not repo:
        return {'available': False, 'reason': 'git のリポジトリが見つかりません', 'days': []}
    try:
        head = _git(repo, 'rev-parse', 'HEAD').strip()
        if _cache.get('key') == (repo, head):
            return _cache['data']
        try:
            url = repo_url(_git(repo, 'remote', 'get-url', 'origin'))
        except subprocess.SubprocessError:
            url = ''
        entries = [_entry(repo, url, sha, parents.split(), stamp, subject, body)
                   for sha, parents, stamp, subject, body in _log_records(repo)]
    except (OSError, subprocess.SubprocessError, ValueError) as e:
        return {'available': False, 'reason': f'git の記録を読めません: {e}', 'days': []}
    data = {'available': True, 'repo_url': url, 'head': head[:7], 'days': _days(entries)}
    _cache.update(key=(repo, head), data=data)
    return data


@contextmanager
def _open_db(db_path):
    path = os.path.expanduser(db_path)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    db = sqlite3.connect(path, timeout=10)
    db.execute('CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT)')
    row = db.execute("SELECT value FROM meta WHERE key = 'schema'").fetchone()
    if not row or row[0] != SCHEMA:
        db.execute('DROP TABLE IF EXISTS entries')
        db.execute('DELETE FROM meta')
        db.execute("INSERT INTO meta VALUES ('schema', ?)", (SCHEMA,))
    db.execute('CREATE TABLE IF NOT EXISTS entries (sha TEXT PRIMARY KEY, ord INTEGER, date TEXT, '
               'tags TEXT, data TEXT)')
    db.execute('CREATE INDEX IF NOT EXISTS entries_date ON entries (date, ord)')
    try:
        with db:
            yield db
    finally:
        db.close()


def sync_dev_diary(repo, db_path):
    """DB を main の今の版に合わせる。増えたコミットだけ _entry を作る。足した件数を返す。"""
    with _lock, _open_db(db_path) as db:
        head = _git(repo, 'rev-parse', 'HEAD').strip()
        meta = dict(db.execute('SELECT key, value FROM meta'))
        if meta.get('head') == head and meta.get('repo') == repo:
            return 0
        try:
            url = repo_url(_git(repo, 'remote', 'get-url', 'origin'))
        except subprocess.SubprocessError:
            url = ''
        if meta.get('repo') != repo or meta.get('url') != url:
            db.execute('DELETE FROM entries')
        known = {sha for (sha,) in db.execute('SELECT sha FROM entries')}
        records = list(_log_records(repo))
        added = 0
        for i, (sha, parents, stamp, subject, body) in enumerate(records):
            ord_ = len(records) - i   # 古いほど小さい（新しい版が入っても変わらない）
            if sha in known:
                db.execute('UPDATE entries SET ord = ? WHERE sha = ?', (ord_, sha))
                continue
            entry = _entry(repo, url, sha, parents.split(), stamp, subject, body)
            db.execute('INSERT INTO entries VALUES (?, ?, ?, ?, ?)',
                       (sha, ord_, entry['date'], json.dumps(entry['tags'], ensure_ascii=False),
                        json.dumps(entry, ensure_ascii=False)))
            added += 1
        current = {r[0] for r in records}
        db.executemany('DELETE FROM entries WHERE sha = ?', [(s,) for s in known - current])
        db.executemany('INSERT OR REPLACE INTO meta VALUES (?, ?)',
                       [('head', head), ('repo', repo), ('url', url)])
        return added


def query_dev_diary(repo, db_path, days=7, before='', tag=''):
    """DB から、before より前の日付の新しい順に days 日分を返す。tag があればその分類だけ。"""
    if not repo:
        return {'available': False, 'reason': 'git のリポジトリが見つかりません', 'days': []}
    try:
        sync_dev_diary(repo, db_path)
        with _lock, _open_db(db_path) as db:
            meta = dict(db.execute('SELECT key, value FROM meta'))
            where, args = ['1'], []
            if tag:
                where.append('tags LIKE ?')
                args.append('%' + json.dumps(tag, ensure_ascii=False) + '%')
            total = db.execute(f'SELECT COUNT(*), COUNT(DISTINCT date) FROM entries WHERE {" AND ".join(where)}',
                               args).fetchone()
            if before:
                where.append('date < ?')
                args.append(before)
            cond = ' AND '.join(where)
            dates = [d for (d,) in db.execute(f'SELECT DISTINCT date FROM entries WHERE {cond} '
                                              'ORDER BY date DESC LIMIT ?', args + [max(1, int(days)) + 1])]
            more = len(dates) > max(1, int(days))
            dates = dates[:max(1, int(days))]
            entries = []
            if dates:
                rows = db.execute(f'SELECT data FROM entries WHERE {cond} AND date >= ? ORDER BY ord DESC',
                                  args + [dates[-1]])
                entries = [json.loads(r[0]) for r in rows]
    except (OSError, subprocess.SubprocessError, ValueError, sqlite3.Error) as e:
        return {'available': False, 'reason': f'開発日記を読めません: {e}', 'days': []}
    return {
        'available': True,
        'repo_url': meta.get('url', ''),
        'head': meta.get('head', '')[:7],
        'days': _days(entries),
        'more': more,
        'next_before': dates[-1] if more else '',
        'total_entries': total[0],
        'total_days': total[1],
    }
