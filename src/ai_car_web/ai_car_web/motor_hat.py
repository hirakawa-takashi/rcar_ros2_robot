"""Adafruit Motor HAT のモーター端子（M1〜M4）と車輪位置の割り付け読み込み。

`motor_hat.yaml` の割り付けに端子ごとの固定情報（PCA9685 チャンネル・TB6612 の
ブリッジ）を重ねてダッシュボードとモーター制御ノードへ返す。
"""

import os

import yaml

# 端子名 -> (PCA9685 PWM ch, IN1 ch, IN2 ch, TB6612 ブリッジ)
_CHANNELS = {
    'M1': (8, 10, 9, 'TB6612 #1 A'),
    'M2': (13, 11, 12, 'TB6612 #1 B'),
    'M3': (2, 4, 3, 'TB6612 #2 A'),
    'M4': (7, 5, 6, 'TB6612 #2 B'),
}

# 車輪位置 -> (表示名, 機体座標 x 前+, y 左+)
WHEELS = {
    'front_left': ('左前', 1, 1),
    'front_right': ('右前', 1, -1),
    'rear_left': ('左後', -1, 1),
    'rear_right': ('右後', -1, -1),
}


def load_motor_hat(path):
    """割り付け YAML を読み、ダッシュボード表示用の辞書を返す。"""
    if not path or not os.path.exists(path):
        error = f'{path} が見つかりません' if path else '割り付け設定ファイル未指定'
        return {'config_path': path, 'error': error, 'hat': {}, 'motors': [], 'warnings': []}
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}

    hat = dict(data.get('hat') or {})
    addr = hat.get('i2c_address')
    if isinstance(addr, int):
        hat['i2c_address'] = f'0x{addr:02X}'

    warnings = []
    motors = []
    seen_channels = set()
    seen_wheels = set()
    for entry in data.get('motors') or []:
        channel = str(entry.get('channel') or '').upper()
        wheel = str(entry.get('wheel') or '')
        if channel not in _CHANNELS:
            warnings.append(f'不明な端子名: {channel or "(空)"}')
            continue
        if wheel not in WHEELS:
            warnings.append(f'{channel}: 不明な車輪位置 {wheel or "(空)"}')
        if channel in seen_channels:
            warnings.append(f'{channel} が重複しています')
        if wheel in seen_wheels:
            warnings.append(f'{wheel} に複数の端子が割り付けられています')
        seen_channels.add(channel)
        seen_wheels.add(wheel)
        pwm, in1, in2, bridge = _CHANNELS[channel]
        label, x, y = WHEELS.get(wheel, ('', 0, 0))
        colors = entry.get('colors') or []
        motors.append({
            'channel': channel,
            'wheel': wheel,
            'wheel_label': label,
            'position': {'x': x, 'y': y},
            'reversed': bool(entry.get('reversed', False)),
            'colors': [str(c) for c in colors if c],
            'note': entry.get('note') or '',
            'pwm_channel': pwm,
            'in1_channel': in1,
            'in2_channel': in2,
            'bridge': bridge,
        })
    for wheel in WHEELS:
        if wheel not in seen_wheels:
            warnings.append(f'{WHEELS[wheel][0]}（{wheel}）に端子が割り付けられていません')

    return {
        'config_path': path,
        'error': None,
        'hat': hat,
        'motors': motors,
        'warnings': warnings,
    }
