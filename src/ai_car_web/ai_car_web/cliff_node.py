#!/usr/bin/env python3
"""落下防止センサー（VL53L1X ×2、前後バンパー）を読み、段差の有無を publish するノード。

センサーは真下から進行方向へ tilt_deg（45°）傾けて床を測る。測った距離を
センサーごとの平らな床の距離（{front,rear}_floor_distance_m）とくらべて、床を 0 cm と
した高さ height_cm = (床の距離 - 測った距離) × cos(tilt) に直す。床より
cliff_height_cm 以上低い（または床が見えない）か、bump_height_cm 以上高い（盛り上がり）
状態が confirm_count 回続くと「段差」と判定し、/cliff_status（std_msgs/String の JSON）で配信する。

VL53L1X の初期アドレス 0x29 は BNO055 と同じなので、XSHUT を GPIO で
1 台ずつ上げ、起動直後に 0x30 / 0x31 などへ変更してから測定する。
wired: false の間は I2C / GPIO に触らず、「未配線」の状態だけを配信する。

平らな床の距離は、平らな床の上で floor_calibrate_service（std_srvs/Trigger）を呼ぶと
floor_calibration_s 秒の中央値で測り直し、floor_calibration_file に保存して次の起動でも使う。

レジスタ設定は ST の VL53L1X ULD と Adafruit_CircuitPython_VL53L1X（MIT）に基づく。
"""

import fcntl
import json
import math
import os
import struct
import time

try:
    import gpiod
except ImportError:
    gpiod = None

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

I2C_SLAVE = 0x0703
REG_I2C_SLAVE_DEVICE_ADDRESS = 0x0001
REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008
REG_GPIO_HV_MUX_CTRL = 0x0030
REG_GPIO_TIO_HV_STATUS = 0x0031
REG_PHASECAL_CONFIG_TIMEOUT_MACROP = 0x004B
REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI = 0x005E
REG_RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060
REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI = 0x0061
REG_RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063
REG_RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069
REG_INTERMEASUREMENT_MS = 0x006C
REG_SD_CONFIG_WOI_SD0 = 0x0078
REG_SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A
REG_SYSTEM_INTERRUPT_CLEAR = 0x0086
REG_SYSTEM_MODE_START = 0x0087
REG_RESULT_RANGE_STATUS = 0x0089
REG_RESULT_RANGE_MM = 0x0096
REG_RESULT_OSC_CALIBRATE_VAL = 0x00DE
REG_MODEL_ID = 0x010F
MODEL_ID = (0xEA, 0xCC)
RANGE_STATUS_VALID = 0x09

# 0x2D〜0x87 の既定設定（ULD の VL51L1X_DEFAULT_CONFIGURATION）
DEFAULT_CONFIGURATION = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
])

# 短距離モード（〜1.3 m）の測定時間 [ms] -> (TIMEOUT_MACROP_A_HI, TIMEOUT_MACROP_B_HI)
TIMING_BUDGET_SHORT = {
    15: (0x001D, 0x0027),
    20: (0x0051, 0x006E),
    33: (0x00D6, 0x006E),
    50: (0x01AE, 0x01E8),
    100: (0x02E1, 0x0388),
    200: (0x03E1, 0x0496),
    500: (0x0591, 0x05C1),
}

SENSORS = ('front', 'rear')
LABELS = {'front': '前', 'rear': '後'}


class VL53L1X:
    """/dev/i2c-N 経由で VL53L1X 1 台を読む（16 ビットのレジスタアドレス）。"""

    def __init__(self, bus: int, address: int):
        self.address = address
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
        except OSError:
            self.close()
            raise
        self._ready_level = 1

    def close(self):
        fd, self.fd = self.fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def write(self, reg: int, data: bytes):
        payload = struct.pack('>H', reg) + data
        if os.write(self.fd, payload) != len(payload):
            raise OSError(f'I2C 書き込み失敗: 0x{reg:04x}')

    def read(self, reg: int, size: int) -> bytes:
        if os.write(self.fd, struct.pack('>H', reg)) != 2:
            raise OSError(f'I2C レジスタ指定失敗: 0x{reg:04x}')
        data = os.read(self.fd, size)
        if len(data) != size:
            raise OSError(f'I2C 読み取り長不足: 0x{reg:04x}')
        return data

    def initialize(self, timing_budget_ms: int, intermeasurement_ms: int):
        model = tuple(self.read(REG_MODEL_ID, 2))
        if model != MODEL_ID:
            raise OSError(f'VL53L1X ではありません（MODEL_ID {model}）')
        self.write(0x002D, DEFAULT_CONFIGURATION)
        polarity = (self.read(REG_GPIO_HV_MUX_CTRL, 1)[0] >> 4) & 0x01
        self._ready_level = 0 if polarity else 1
        self.start()
        deadline = time.monotonic() + 1.0
        while not self.data_ready():
            if time.monotonic() > deadline:
                raise OSError('VL53L1X の初回測定がタイムアウトしました')
            time.sleep(0.01)
        self.clear_interrupt()
        self.stop()
        self.write(REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, b'\x09')
        self.write(0x000B, b'\x00')
        self._set_short_mode()
        a_hi, b_hi = TIMING_BUDGET_SHORT[timing_budget_ms]
        self.write(REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, struct.pack('>H', a_hi))
        self.write(REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, struct.pack('>H', b_hi))
        clock_pll = struct.unpack('>H', self.read(REG_RESULT_OSC_CALIBRATE_VAL, 2))[0] & 0x03FF
        period = int(clock_pll * max(intermeasurement_ms, timing_budget_ms) * 1.075)
        self.write(REG_INTERMEASUREMENT_MS, struct.pack('>I', period))
        self.start()

    def _set_short_mode(self):
        self.write(REG_PHASECAL_CONFIG_TIMEOUT_MACROP, b'\x14')
        self.write(REG_RANGE_CONFIG_VCSEL_PERIOD_A, b'\x07')
        self.write(REG_RANGE_CONFIG_VCSEL_PERIOD_B, b'\x05')
        self.write(REG_RANGE_CONFIG_VALID_PHASE_HIGH, b'\x38')
        self.write(REG_SD_CONFIG_WOI_SD0, b'\x07\x05')
        self.write(REG_SD_CONFIG_INITIAL_PHASE_SD0, b'\x06\x06')

    def start(self):
        self.write(REG_SYSTEM_MODE_START, b'\x40')

    def stop(self):
        self.write(REG_SYSTEM_MODE_START, b'\x00')

    def clear_interrupt(self):
        self.write(REG_SYSTEM_INTERRUPT_CLEAR, b'\x01')

    def data_ready(self) -> bool:
        return (self.read(REG_GPIO_TIO_HV_STATUS, 1)[0] & 0x01) == self._ready_level

    def read_range(self):
        """(有効か, 距離 [m]) を返し、次の測定のために割り込みを解除する。"""
        status = self.read(REG_RESULT_RANGE_STATUS, 1)[0] & 0x1F
        mm = struct.unpack('>H', self.read(REG_RESULT_RANGE_MM, 2))[0]
        self.clear_interrupt()
        return status == RANGE_STATUS_VALID, mm / 1000.0


def set_address_blind(bus: int, old_address: int, new_address: int):
    """読み出しをせずにアドレス変更だけを書く（0x29 を BNO055 と共有しているため）。"""
    fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
    try:
        fcntl.ioctl(fd, I2C_SLAVE, old_address)
        payload = struct.pack('>HB', REG_I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)
        if os.write(fd, payload) != len(payload):
            raise OSError('VL53L1X のアドレス変更に失敗しました')
    finally:
        os.close(fd)


class CliffNode(Node):
    """前後の VL53L1X で段差（床の途切れ）を判定し、/cliff_status に publish する。"""

    def __init__(self):
        super().__init__('cliff_node')
        self.declare_parameter('wired', False)
        self.declare_parameter('front_wired', True)
        self.declare_parameter('rear_wired', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('gpio_chip', '')
        self.declare_parameter('default_address', 0x29)
        self.declare_parameter('front_address', 0x30)
        self.declare_parameter('rear_address', 0x31)
        self.declare_parameter('front_xshut_gpio', 17)
        self.declare_parameter('rear_xshut_gpio', 27)
        self.declare_parameter('status_topic', '/cliff_status')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('timing_budget_ms', 33)
        self.declare_parameter('intermeasurement_ms', 40)
        self.declare_parameter('tilt_deg', 45.0)
        self.declare_parameter('front_floor_distance_m', 0.087)
        self.declare_parameter('rear_floor_distance_m', 0.082)
        self.declare_parameter('cliff_height_cm', 2.0)
        self.declare_parameter('bump_height_cm', 2.0)
        self.declare_parameter('invalid_is_cliff', True)
        self.declare_parameter('confirm_count', 2)
        self.declare_parameter('stale_timeout', 0.5)
        self.declare_parameter('floor_calibrate_service', '/cliff/floor_calibrate')
        self.declare_parameter('floor_calibration_file', '~/.ros/cliff_floor_calibration.json')
        self.declare_parameter('floor_calibration_s', 3.0)
        self.declare_parameter('floor_calibration_max_change_m', 0.03)
        self.declare_parameter('floor_calibration_max_spread_m', 0.02)

        wired = bool(self.get_parameter('wired').value)
        self.active = [s for s in SENSORS if wired and self.get_parameter(f'{s}_wired').value]
        self.wired = bool(self.active)
        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.default_address = int(self.get_parameter('default_address').value)
        self.addresses = {s: int(self.get_parameter(f'{s}_address').value) for s in SENSORS}
        self.xshut = {s: int(self.get_parameter(f'{s}_xshut_gpio').value) for s in SENSORS}
        budget = int(self.get_parameter('timing_budget_ms').value)
        self.timing_budget = budget if budget in TIMING_BUDGET_SHORT else 33
        self.intermeasurement = int(self.get_parameter('intermeasurement_ms').value)
        self.tilt_deg = float(self.get_parameter('tilt_deg').value)
        self.cos_tilt = math.cos(math.radians(self.tilt_deg))
        self.floor_distance = {
            s: float(self.get_parameter(f'{s}_floor_distance_m').value) for s in SENSORS}
        self.cliff_height_cm = float(self.get_parameter('cliff_height_cm').value)
        self.bump_height_cm = float(self.get_parameter('bump_height_cm').value)
        self.invalid_is_cliff = bool(self.get_parameter('invalid_is_cliff').value)
        self.confirm_count = max(1, int(self.get_parameter('confirm_count').value))
        self.stale_timeout = float(self.get_parameter('stale_timeout').value)
        self.default_floor = dict(self.floor_distance)
        self.calibration_file = os.path.expanduser(
            str(self.get_parameter('floor_calibration_file').value))
        self.calibration_s = float(self.get_parameter('floor_calibration_s').value)
        self.calibration_max_change = float(self.get_parameter('floor_calibration_max_change_m').value)
        self.calibration_max_spread = float(self.get_parameter('floor_calibration_max_spread_m').value)
        self._calibrated_at = ''
        self._cal_rec = None
        self._cal_result = ''
        self._load_floor_calibration()

        self.pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10)
        self._chip = None
        self._lines = {}
        self._sensors = {}
        self._state = {s: self._empty_state() for s in SENSORS}
        self._error = None
        self._next_retry = 0.0
        self._last_error_log = 0.0

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)
        service = str(self.get_parameter('floor_calibrate_service').value)
        if service:
            self.create_service(Trigger, service, self._start_floor_calibration)
        if not self.wired:
            self.get_logger().info('落下防止センサーは未配線（wired: false）のため状態だけを配信します')

    @staticmethod
    def _empty_state():
        return {'distance_m': None, 'height_cm': None, 'valid': False, 'level': 'unknown',
                'over_count': 0, 'stamp': 0.0}

    # --- 床の補正 ---
    def _floor_ok(self, side: str, distance: float) -> bool:
        return abs(distance - self.default_floor[side]) <= self.calibration_max_change

    def _load_floor_calibration(self):
        try:
            with open(self.calibration_file, encoding='utf-8') as f:
                data = json.load(f)
            floors = {s: float(data[f'{s}_floor_distance_m']) for s in SENSORS
                      if data.get(f'{s}_floor_distance_m') is not None}
        except FileNotFoundError:
            return
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().warn(f'床の補正を読みこめません: {e}')
            return
        for side, distance in floors.items():
            if self._floor_ok(side, distance):
                self.floor_distance[side] = distance
            else:
                self.get_logger().warn(
                    f'床の補正の{LABELS[side]} {distance:.3f} m は設定 {self.default_floor[side]:.3f} m から'
                    f' {self.calibration_max_change * 100:.0f} cm より離れているので使いません')
        self._calibrated_at = str(data.get('stamp', ''))
        self.get_logger().info(
            f'床の補正を読みこみました: {self.calibration_file}（' + '・'.join(
                f'{LABELS[s]} {self.floor_distance[s]:.3f} m' for s in SENSORS) + '）')

    def _start_floor_calibration(self, request, response):
        if not self._sensors:
            response.success = False
            response.message = '落下防止センサーがつながっていません'
        elif self._cal_rec is not None:
            response.success = False
            response.message = '床の補正の記録中です'
        else:
            self._cal_rec = {'start': time.monotonic(),
                             'samples': {s: [] for s in self.active},
                             'invalid': {s: 0 for s in self.active}}
            self._cal_result = ''
            response.success = True
            response.message = f'{self.calibration_s:.0f} 秒のあいだ、車を平らな床に置いたまま動かさないでください'
            self.get_logger().info('床の補正の記録を始めました')
        return response

    def _record_floor(self, side: str, valid: bool, distance: float):
        rec = self._cal_rec
        if rec is None or side not in rec['samples']:
            return
        if valid:
            rec['samples'][side].append(distance)
        else:
            rec['invalid'][side] += 1

    def _finish_floor_calibration(self):
        rec, self._cal_rec = self._cal_rec, None
        new = {}
        for side in rec['samples']:
            samples = sorted(rec['samples'][side])
            label = LABELS[side]
            if rec['invalid'][side] or len(samples) < 10:
                self._cal_result = f'{label}のセンサーが床を測れません（測れた {len(samples)} 回・無効 {rec["invalid"][side]} 回）'
                return
            spread = samples[-1] - samples[0]
            if spread > self.calibration_max_spread:
                self._cal_result = f'{label}の距離のゆれが {spread * 1000:.0f} mm で大きすぎます（車を動かさないでください）'
                return
            median = samples[len(samples) // 2]
            if not self._floor_ok(side, median):
                self._cal_result = (f'{label}の床までの距離 {median * 1000:.0f} mm が設定 {self.default_floor[side] * 1000:.0f} mm から'
                                    f' {self.calibration_max_change * 1000:.0f} mm より離れています（平らな床に置いてください）')
                return
            new[side] = round(median, 4)
        stamp = time.strftime('%Y-%m-%d %H:%M')
        data = {f'{s}_floor_distance_m': new.get(s, self.floor_distance[s]) for s in SENSORS}
        data['stamp'] = stamp
        tmp = self.calibration_file + '.tmp'
        try:
            os.makedirs(os.path.dirname(self.calibration_file) or '.', exist_ok=True)
            with open(tmp, 'w', encoding='utf-8') as f:
                json.dump(data, f)
            os.replace(tmp, self.calibration_file)
        except OSError as e:
            self._cal_result = f'床の補正を保存できません: {e}'
            return
        old = dict(self.floor_distance)
        self.floor_distance.update(new)
        self._calibrated_at = stamp
        self._cal_result = '補正しました（' + '・'.join(
            f'{LABELS[s]} {old[s] * 1000:.0f} → {new[s] * 1000:.0f} mm' for s in new) + '）'
        self.get_logger().info(self._cal_result)

    def _calibration_payload(self):
        rec = self._cal_rec
        return {
            'recording': rec is not None,
            'remaining_s': round(max(0.0, self.calibration_s - (time.monotonic() - rec['start'])), 1) if rec else 0.0,
            'calibrated_at': self._calibrated_at,
            'result': self._cal_result,
        }

    # --- 接続 ---
    def _find_chip(self):
        if self.gpio_chip:
            return gpiod.Chip(self.gpio_chip)
        for index in range(10):
            try:
                chip = gpiod.Chip(f'/dev/gpiochip{index}')
            except OSError:
                continue
            if chip.label() == 'pinctrl-rp1':
                return chip
            if hasattr(chip, 'close'):
                chip.close()
        raise OSError('pinctrl-rp1 の gpiochip が見つかりません')

    def _connect(self):
        if gpiod is None:
            raise OSError('python3-libgpiod がありません')
        self._hold_xshut_low()
        time.sleep(0.01)
        for side in self.active:
            self._lines[side].set_value(1)
            # 起動（最大 1.2 ms）を待ってから、0x29 のうちにアドレスを変える
            time.sleep(0.01)
            set_address_blind(self.i2c_bus, self.default_address, self.addresses[side])
            sensor = VL53L1X(self.i2c_bus, self.addresses[side])
            try:
                sensor.initialize(self.timing_budget, self.intermeasurement)
            except OSError:
                sensor.close()
                raise
            self._sensors[side] = sensor
        now = time.monotonic()
        for side in self.active:
            self._state[side] = self._empty_state()
            self._state[side]['stamp'] = now
        self._error = None
        self.get_logger().info(
            'VL53L1X 接続: ' + ', '.join(
                f'{LABELS[s]} 0x{self.addresses[s]:02x}（XSHUT GPIO{self.xshut[s]}）'
                for s in self.active))

    def _hold_xshut_low(self):
        """XSHUT を L で握ったままにする。

        XSHUT は基板で引き上げられていて、離すとセンサーが 0x29（BNO055 と同じ）で起きる。
        """
        if self._chip is None:
            self._chip = self._find_chip()
        for side in self.active:
            if side not in self._lines:
                line = self._chip.get_line(self.xshut[side])
                line.request(consumer='cliff_node', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
                self._lines[side] = line
            self._lines[side].set_value(0)

    def _disconnect(self, release=False):
        for sensor in self._sensors.values():
            try:
                sensor.stop()
            except OSError:
                pass
            sensor.close()
        self._sensors = {}
        for line in self._lines.values():
            try:
                line.set_value(0)
                if release:
                    line.release()
            except OSError:
                pass
        if release:
            self._lines = {}
            if self._chip is not None and hasattr(self._chip, 'close'):
                self._chip.close()
            self._chip = None
        for side in SENSORS:
            self._state[side] = self._empty_state()

    def _warn(self, message: str, error: Exception):
        self._error = f'{message}: {error}'
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            self.get_logger().warn(self._error)
            self._last_error_log = now

    # --- 判定 ---
    def _update(self, side: str, valid: bool, distance: float):
        self._record_floor(side, valid, distance)
        st = self._state[side]
        height_cm = (self.floor_distance[side] - distance) * self.cos_tilt * 100.0
        bump = valid and self.bump_height_cm > 0 and height_cm >= self.bump_height_cm
        over = (height_cm <= -self.cliff_height_cm or bump) if valid else self.invalid_is_cliff
        st['over_count'] = st['over_count'] + 1 if over else 0
        st['bump'] = bump
        st['valid'] = valid
        st['distance_m'] = round(distance, 3) if valid else None
        st['height_cm'] = round(height_cm, 1) if valid else None
        st['level'] = 'cliff' if st['over_count'] >= self.confirm_count else 'floor'
        st['stamp'] = time.monotonic()

    def _poll(self):
        now = time.monotonic()
        for side, sensor in self._sensors.items():
            if sensor.data_ready():
                self._update(side, *sensor.read_range())
            elif now - self._state[side]['stamp'] > self.stale_timeout:
                raise OSError(f'{LABELS[side]}のセンサーから測定値が来ません')

    def _payload(self):
        sensors = {}
        for side in SENSORS:
            st = self._state[side]
            if side not in self.active:
                level = 'unwired'
            elif not self._sensors:
                level = 'unknown'
            else:
                level = st['level']
            sensors[side] = {
                'label': LABELS[side],
                'address': f'0x{self.addresses[side]:02x}',
                'xshut_gpio': self.xshut[side],
                'level': level,
                'distance_m': st['distance_m'] if level in ('floor', 'cliff') else None,
                'height_cm': st['height_cm'] if level in ('floor', 'cliff') else None,
                'floor_distance_m': self.floor_distance[side],
                'valid': st['valid'] if level in ('floor', 'cliff') else False,
                'bump': level == 'cliff' and bool(st.get('bump')),
            }
        cliff = [s for s in SENSORS if sensors[s]['level'] == 'cliff']
        if not self.wired:
            level, reason = 'unwired', '未配線（wired: false）'
        elif not self._sensors:
            level, reason = 'unknown', self._error or '接続待ち'
        elif cliff:
            level = 'cliff'
            reason = '・'.join(
                LABELS[s] + ('に盛り上がり' if sensors[s]['bump'] else 'に段差') for s in cliff)
        else:
            level, reason = 'floor', '床あり'
            unwired = [LABELS[s] for s in SENSORS if s not in self.active]
            if unwired:
                reason += f"（{'・'.join(unwired)}は未配線）"
        return {
            'stamp': round(time.time(), 3),
            'wired': self.wired,
            'connected': bool(self._sensors),
            'level': level,
            'reason': reason,
            'block_forward': sensors['front']['level'] == 'cliff',
            'block_backward': sensors['rear']['level'] == 'cliff',
            'tilt_deg': self.tilt_deg,
            'cliff_height_cm': self.cliff_height_cm,
            'bump_height_cm': self.bump_height_cm,
            'floor_calibration': self._calibration_payload(),
            'sensors': sensors,
        }

    def _timer_cb(self):
        if self.wired:
            now = time.monotonic()
            if not self._sensors and now >= self._next_retry:
                try:
                    self._connect()
                except Exception as e:
                    self._disconnect()
                    self._warn('VL53L1X に接続できません', e)
                    self._next_retry = now + 5.0
            if self._sensors:
                try:
                    self._poll()
                except OSError as e:
                    self._disconnect()
                    self._warn('VL53L1X 読み取りエラー', e)
                    self._next_retry = time.monotonic() + 2.0
            if self._cal_rec is not None:
                if not self._sensors:
                    self._cal_rec = None
                    self._cal_result = '記録の途中でセンサーが切れました'
                elif time.monotonic() - self._cal_rec['start'] >= self.calibration_s:
                    self._finish_floor_calibration()
        msg = String()
        msg.data = json.dumps(self._payload(), ensure_ascii=False)
        self.pub.publish(msg)

    def destroy_node(self):
        self._disconnect(release=True)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CliffNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
