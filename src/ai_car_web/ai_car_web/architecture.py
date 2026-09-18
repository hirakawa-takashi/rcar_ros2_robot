"""プロジェクト説明画面の構成定義読み込み。"""

import os
import time

import yaml


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


def load_architecture(path, params):
    """構成 YAML を読み、参照パラメータを実値に解決して返す。"""
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
    result['generated_at'] = time.time()
    return result
