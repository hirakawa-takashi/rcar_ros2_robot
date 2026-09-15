"""Adafruit Motor HAT のモーター端子（M1〜M4）と車輪位置の割り付け読み込み。

`motor_hat.yaml` の割り付けに端子ごとの固定情報（PCA9685 チャンネル・TB6612 の
ブリッジ）を重ねてダッシュボードとモーター制御ノードへ返す。
"""

import os

import yaml

from ai_car_web.gpio_pinout import _HEADER

# 物理ピン -> (名称, BCM, 種別)
_PINS = {pin: (name, bcm, kind) for pin, name, bcm, kind in _HEADER}

# 端子名 -> (PCA9685 PWM ch, IN1 ch, IN2 ch, TB6612 ブリッジ)
_CHANNELS = {
    'M1': (8, 10, 9, 'TB6612 #1 A'),
    'M2': (13, 11, 12, 'TB6612 #1 B'),
    'M3': (2, 4, 3, 'TB6612 #2 A'),
    'M4': (7, 5, 6, 'TB6612 #2 B'),
}

# 車輪位置 -> (表示名, 機体座標 x 前+, y 左+)
WHEELS = {
    'front_left': ('前左', 1, 1),
    'front_right': ('前右', 1, -1),
    'rear_left': ('後左', -1, 1),
    'rear_right': ('後右', -1, -1),
}


def _encoder_pin(pin, label, seen, warnings):
    """エンコーダー信号ピンを検証し {pin, bcm, name} を返す（未設定なら None）。"""
    if pin is None:
        return None
    info = _PINS.get(pin)
    if info is None or info[1] is None:
        warnings.append(f'{label}: 物理ピン {pin} は GPIO ではありません')
        return {'pin': pin, 'bcm': None, 'name': info[0] if info else ''}
    if info[2] in ('i2c', 'id'):
        warnings.append(f'{label}: ピン {pin} ({info[0]}) は I2C/ID 用です')
    if pin in seen:
        warnings.append(f'{label}: ピン {pin} が {seen[pin]} と重複しています')
    seen[pin] = label
    return {'pin': pin, 'bcm': info[1], 'name': info[0]}


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
    seen_pins = {}
    enc_common = dict(hat.get('encoder') or {})
    if enc_common:
        vcc = _PINS.get(enc_common.get('vcc_pin'))
        if vcc and vcc[2] != 'power3v3':
            warnings.append(f'エンコーダー VCC ピン {enc_common["vcc_pin"]} は 3V3 ではありません')
        enc_common['gnd_pins'] = list(enc_common.get('gnd_pins') or [])
        enc_common['colors'] = [str(c) for c in enc_common.get('colors') or [] if c]
        hat['encoder'] = enc_common
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
        enc = entry.get('encoder') or {}
        encoder = None
        if enc:
            encoder = {
                'a': _encoder_pin(enc.get('a_pin'), f'{channel} ENC A', seen_pins, warnings),
                'b': _encoder_pin(enc.get('b_pin'), f'{channel} ENC B', seen_pins, warnings),
                'colors': [str(c) for c in enc.get('colors') or [] if c],
            }
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
            'encoder': encoder,
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
