"""プロジェクト説明画面の構成定義読み込み。"""

import os
import time

import yaml

VIDEO_DIR = 'videos'


def _resolve_refs(value, params):
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            if key == 'ref':
                result[key] = item
                continue
            result[key] = _resolve_refs(item, params)
        ref = value.get('ref')
        if isinstance(ref, str) and '.' in ref:
            node, param = ref.split('.', 1)
            resolved = (params.get(node) or {}).get(param)
            result['value'] = (('(設定あり)' if resolved else '(未設定)')
                               if value.get('secret') else resolved)
        return result
    if isinstance(value, list):
        return [_resolve_refs(item, params) for item in value]
    return value


def _repo_file(repo_dir, rel_path):
    if not repo_dir or not rel_path:
        return None
    root = os.path.realpath(repo_dir)
    path = os.path.realpath(os.path.join(root, rel_path))
    if os.path.commonpath([root, path]) != root:
        return None
    return path


def count_lines(repo_dir, rel_path):
    """リポジトリの中のファイルの行数（wc -l と同じ数え方）。数えられなければ None。"""
    path = _repo_file(repo_dir, rel_path)
    if not path:
        return None
    try:
        with open(path, 'rb') as f:
            return f.read().count(b'\n')
    except OSError:
        return None


def read_program(path, repo_dir, rel_path, max_bytes=1_000_000):
    """programs に登録したファイルだけ、中身を返す。登録外・読めないときは None。"""
    try:
        with open(path, encoding='utf-8') as f:
            programs = (yaml.safe_load(f) or {}).get('programs') or []
    except (OSError, yaml.YAMLError, AttributeError):
        return None
    program = next((p for p in programs
                    if isinstance(p, dict) and p.get('file') == rel_path), None)
    file_path = _repo_file(repo_dir, rel_path) if program else None
    if not file_path:
        return None
    try:
        with open(file_path, 'rb') as f:
            data = f.read(max_bytes + 1)
    except OSError:
        return None
    return {
        'name': program.get('name'),
        'place': program.get('place'),
        'file': rel_path,
        'text': data[:max_bytes].decode('utf-8', errors='replace'),
        'truncated': len(data) > max_bytes,
    }


def _add_line_counts(programs, repo_dir):
    for program in programs:
        if isinstance(program, dict):
            program['lines'] = count_lines(repo_dir, program.get('file'))


def _add_videos(programs, repo_dir):
    """video に書いた解説動画が repo_dir/videos にあれば、video_url（/videos/…）を入れる。"""
    for program in programs:
        if not isinstance(program, dict):
            continue
        name = program.get('video')
        path = _repo_file(repo_dir, os.path.join(VIDEO_DIR, name)) if isinstance(name, str) else None
        ok = bool(path) and os.path.dirname(path) == _repo_file(repo_dir, VIDEO_DIR) \
            and os.path.isfile(path)
        program['video_url'] = f'/videos/{name}' if ok else None


def load_architecture(path, params, repo_dir=None):
    """構成 YAML を読み、参照パラメータを実値に解決して返す。

    repo_dir を渡すと、programs の各ファイルの行数を lines に、解説動画があれば video_url に入れる。
    """
    if not path or not os.path.exists(path):
        error = f'{path} が見つかりません' if path else '構成設定ファイル未指定'
        return {'error': error}
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError) as exc:
        return {'error': f'{path} の読み込みに失敗しました: {exc}'}
    if not isinstance(data, dict):
        return {'error': f'{path} のルート要素がマッピングではありません'}
    result = _resolve_refs(data, params or {})
    if isinstance(result.get('programs'), list):
        _add_line_counts(result['programs'], repo_dir)
        _add_videos(result['programs'], repo_dir)
    _add_videos([v for k, v in result.items() if k != 'programs' and isinstance(v, dict)], repo_dir)
    result['generated_at'] = time.time()
    return result
