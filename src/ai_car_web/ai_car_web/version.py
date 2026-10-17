"""AI-CAR のバージョン（ai_car_web の package.xml の <version>）と、いま動いている git の版。"""

import os
import re
import subprocess

VERSION_RE = re.compile(r'<version>\s*([^<\s]+)\s*</version>')


def read_package_version(package_xml):
    try:
        with open(package_xml, encoding='utf-8') as f:
            m = VERSION_RE.search(f.read())
    except OSError:
        return ''
    return m.group(1) if m else ''


def _git(repo_dir, *args):
    try:
        return subprocess.run(['git', '-C', repo_dir, *args], capture_output=True, text=True,
                              timeout=5, check=True).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ''


def release_date(changelog, version):
    """CHANGELOG.md の「## [x.y.z] - YYYY-MM-DD」の日付。"""
    try:
        with open(changelog, encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return ''
    m = re.search(rf'^## \[{re.escape(version)}\]\s*-\s*(\d{{4}}-\d{{2}}-\d{{2}})', text, re.M)
    return m.group(1) if m else ''


def load_version(share_dir, repo_dir=None):
    version = read_package_version(os.path.join(share_dir, 'package.xml'))
    info = {'version': version, 'released': '', 'commit': '', 'commit_date': ''}
    if repo_dir:
        info['released'] = release_date(os.path.join(repo_dir, 'CHANGELOG.md'), version) if version else ''
        info['commit'] = _git(repo_dir, 'rev-parse', '--short', 'HEAD')
        info['commit_date'] = _git(repo_dir, 'log', '-1', '--format=%cs')
    return info
