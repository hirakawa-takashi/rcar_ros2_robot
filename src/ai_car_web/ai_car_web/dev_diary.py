"""開発日記: git の main の記録（PR のマージ）と CHANGELOG の追加行から日付ごとの変更一覧を作る。"""

import os
import re
import subprocess
from datetime import datetime, timedelta, timezone

JST = timezone(timedelta(hours=9))
EMPTY_TREE = '4b825dc642cb6eb9a060e54bf8d69288fbee4904'
MERGE_RE = re.compile(r'^Merge pull request #(\d+) from ')
GITHUB_RE = re.compile(r'github\.com[/:]([^/\s]+)/([^/\s]+?)(?:\.git)?/?$')
TRAILER_RE = re.compile(r'^[\w-]+-by:', re.IGNORECASE)
WIRING_FILES = ('gpio_pins.yaml', 'motor_hat.yaml', 'HARDWARE_BOM.md')

_cache = {}


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


def load_dev_diary(repo):
    """main（最初の親をたどる）の記録を、新しい日付順にまとめて返す。"""
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
        log = _git(repo, 'log', '--first-parent', '--format=%H%x1f%P%x1f%cI%x1f%s%x1f%b%x1e')
        days = {}
        for record in log.split('\x1e'):
            fields = record.strip('\n').split('\x1f')
            if len(fields) < 5:
                continue
            sha, parents, stamp, subject, body = fields
            entry = _entry(repo, url, sha, parents.split(), stamp, subject, body)
            days.setdefault(entry['date'], []).append(entry)
    except (OSError, subprocess.SubprocessError, ValueError) as e:
        return {'available': False, 'reason': f'git の記録を読めません: {e}', 'days': []}
    data = {
        'available': True,
        'repo_url': url,
        'head': head[:7],
        'days': [{'date': d, 'entries': days[d]} for d in sorted(days, reverse=True)],
    }
    _cache.update(key=(repo, head), data=data)
    return data
