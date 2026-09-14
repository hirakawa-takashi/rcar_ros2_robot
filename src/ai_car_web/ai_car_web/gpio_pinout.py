"""Raspberry Pi 5 の 40 ピンヘッダー定義と配線設定の読み込み。

物理ピン番号ごとの固定情報（名称・BCM 番号・種別）に、`gpio_pins.yaml` の
配線情報（接続先・信号・配線色）を重ねてダッシュボードへ返す。
"""

import os

import yaml

# (物理ピン, 名称, BCM GPIO 番号, 種別)
# 種別: power5v / power3v3 / ground / i2c / uart / spi / pwm / pcm / id / gpio
_HEADER = [
    (1, '3V3 power', None, 'power3v3'),
    (2, '5V power', None, 'power5v'),
    (3, 'GPIO 2 / SDA', 2, 'i2c'),
    (4, '5V power', None, 'power5v'),
    (5, 'GPIO 3 / SCL', 3, 'i2c'),
    (6, 'Ground', None, 'ground'),
    (7, 'GPIO 4 / GPCLK0', 4, 'gpio'),
    (8, 'GPIO 14 / TXD', 14, 'uart'),
    (9, 'Ground', None, 'ground'),
    (10, 'GPIO 15 / RXD', 15, 'uart'),
    (11, 'GPIO 17', 17, 'gpio'),
    (12, 'GPIO 18 / PCM_CLK', 18, 'pcm'),
    (13, 'GPIO 27', 27, 'gpio'),
    (14, 'Ground', None, 'ground'),
    (15, 'GPIO 22', 22, 'gpio'),
    (16, 'GPIO 23', 23, 'gpio'),
    (17, '3V3 power', None, 'power3v3'),
    (18, 'GPIO 24', 24, 'gpio'),
    (19, 'GPIO 10 / MOSI', 10, 'spi'),
    (20, 'Ground', None, 'ground'),
    (21, 'GPIO 9 / MISO', 9, 'spi'),
    (22, 'GPIO 25', 25, 'gpio'),
    (23, 'GPIO 11 / SCLK', 11, 'spi'),
    (24, 'GPIO 8 / CE0', 8, 'spi'),
    (25, 'Ground', None, 'ground'),
    (26, 'GPIO 7 / CE1', 7, 'spi'),
    (27, 'GPIO 0 / ID_SD', 0, 'id'),
    (28, 'GPIO 1 / ID_SC', 1, 'id'),
    (29, 'GPIO 5', 5, 'gpio'),
    (30, 'Ground', None, 'ground'),
    (31, 'GPIO 6', 6, 'gpio'),
    (32, 'GPIO 12 / PWM0', 12, 'pwm'),
    (33, 'GPIO 13 / PWM1', 13, 'pwm'),
    (34, 'Ground', None, 'ground'),
    (35, 'GPIO 19 / PCM_FS', 19, 'pcm'),
    (36, 'GPIO 16', 16, 'gpio'),
    (37, 'GPIO 26', 26, 'gpio'),
    (38, 'GPIO 20 / PCM_DIN', 20, 'pcm'),
    (39, 'Ground', None, 'ground'),
    (40, 'GPIO 21 / PCM_DOUT', 21, 'pcm'),
]


def load_connections(path):
    """配線設定 YAML を読み、物理ピン番号 -> 配線情報リストの辞書を返す。"""
    if not path or not os.path.exists(path):
        return {}, f'{path} が見つかりません' if path else '配線設定ファイル未指定'
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    by_pin = {}
    for entry in data.get('connections') or []:
        pins = entry.get('pins')
        if pins is None and entry.get('pin') is not None:
            pins = [entry['pin']]
        for pin in pins or []:
            by_pin.setdefault(int(pin), []).append({
                'device': entry.get('device') or '',
                'signal': entry.get('signal') or '',
                'color': entry.get('color') or '',
                'note': entry.get('note') or '',
            })
    return by_pin, None


def build_pinout(config_path):
    """40 ピンの一覧（配線情報を含む）を返す。"""
    connections, error = load_connections(config_path)
    pins = []
    for number, name, bcm, kind in _HEADER:
        conns = connections.get(number, [])
        pins.append({
            'pin': number,
            'name': name,
            'bcm': bcm,
            'kind': kind,
            'in_use': bool(conns),
            'devices': [c['device'] for c in conns if c['device']],
            'signals': [c['signal'] for c in conns if c['signal']],
            'colors': [c['color'] for c in conns if c['color']],
            'notes': [c['note'] for c in conns if c['note']],
        })
    return {
        'board': 'Raspberry Pi 5',
        'config_path': config_path,
        'error': error,
        'used_count': sum(1 for p in pins if p['in_use']),
        'pins': pins,
    }
