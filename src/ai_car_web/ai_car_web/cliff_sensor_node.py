#!/usr/bin/env python3
"""前後バンパーの VL53L1X（落下防止センサー）を読み、段差を /cliff_status に publish するノード。

2 個の VL53L1X と GY-BNO055 はどれも I2C アドレス 0x29 なので、起動時に
XSHUT（GPIO）で 1 個ずつ起こし、0x29 のままでは読まずにアドレスだけ書き換える
（BNO055 の 0x00 番地は読み取り専用で、書き込みは無視される）。

床は 45° 前傾で見ているため、平らな床では「床までの高さ × 1.41」前後の距離になる。
距離が cliff_distance_mm を超えるか、反射が返らない（信号なし・範囲外）状態が
cliff_confirm 回続いたら段差とみなす。clear_confirm 回続けて床が見えたら解除する。
"""

import fcntl
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import gpiod
except ImportError:
    gpiod = None

I2C_SLAVE = 0x0703
DEFAULT_ADDRESS = 0x29

REG_I2C_SLAVE_DEVICE_ADDRESS = 0x0001
REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008
REG_GPIO_TIO_HV_STATUS = 0x0031
REG_PHASECAL_CONFIG_TIMEOUT_MACROP = 0x004B
REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI = 0x005E
REG_RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060
REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI = 0x0061
REG_RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063
REG_RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069
REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x006C
REG_SD_CONFIG_WOI_SD0 = 0x0078
REG_SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A
REG_SYSTEM_INTERRUPT_CLEAR = 0x0086
REG_SYSTEM_MODE_START = 0x0087
REG_RESULT_RANGE_STATUS = 0x0089
REG_RESULT_DISTANCE_MM = 0x0096
REG_RESULT_OSC_CALIBRATE_VAL = 0x00DE
REG_FIRMWARE_SYSTEM_STATUS = 0x00E5
REG_IDENTIFICATION_MODEL_ID = 0x010F

# ST VL53L1X ULD API の VL51L1X_DEFAULT_CONFIGURATION（0x2D〜0x87）
DEFAULT_CONFIGURATION = bytes([
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08,
    0x00, 0x08, 0x10, 0x01, 0x01, 0x00, 0x00, 0x00,
    0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21,
    0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0xC8,
    0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01,
    0x68, 0x00, 0x80, 0x08, 0xB8, 0x00, 0x00, 0x00,
    0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00,
    0x00, 0x02, 0xC7, 0xFF, 0x9B, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00,
])

# 短距離モード（〜1.3 m）のタイミングバジェット [ms] → (A_HI, B_HI)
TIMING_BUDGET_SHORT = {
    15: (0x001D, 0x0027),
    20: (0x0051, 0x006E),
    33: (0x00D6, 0x006E),
    50: (0x01AE, 0x01E8),
    100: (0x02E1, 0x0388),
}

RANGE_VALID = 9
RANGE_MIN_RANGE_FAIL = 8


class VL53L1X:
    """VL53L1X 1 個を /dev/i2c-N から直接読む（ST ULD API と同じ手順）。"""

    def __init__(self, bus: int, address: int):
        self.address = address
        self._fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        try:
            fcntl.ioctl(self._fd, I2C_SLAVE, address)
        except OSError:
            self.close()
            raise

    def close(self):
        fd, self._fd = self._fd, None
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def write(self, reg: int, data: bytes):
        buf = reg.to_bytes(2, 'big') + data
        if os.write(self._fd, buf) != len(buf):
            raise OSError(f'I2C 書き込み失敗: 0x{reg:04x}')

    def read(self, reg: int, size: int) -> bytes:
        if os.write(self._fd, reg.to_bytes(2, 'big')) != 2:
            raise OSError(f'I2C レジスタ指定失敗: 0x{reg:04x}')
        data = os.read(self._fd, size)
        if len(data) != size:
            raise OSError(f'I2C 読み取り長不足: 0x{reg:04x}')
        return data

    def write_u8(self, reg: int, value: int):
        self.write(reg, bytes([value & 0xFF]))

    def write_u16(self, reg: int, value: int):
        self.write(reg, (value & 0xFFFF).to_bytes(2, 'big'))

    def read_u8(self, reg: int) -> int:
        return self.read(reg, 1)[0]

    def read_u16(self, reg: int) -> int:
        return int.from_bytes(self.read(reg, 2), 'big')

    def initialize(self, timing_budget_ms: int, intermeasurement_ms: int):
        model = self.read_u16(REG_IDENTIFICATION_MODEL_ID)
        if model != 0xEACC:
            raise OSError(f'VL53L1X の MODEL_ID が不正です: 0x{model:04x}')
        deadline = time.monotonic() + 0.5
        while not self.read_u8(REG_FIRMWARE_SYSTEM_STATUS) & 0x01:
            if time.monotonic() > deadline:
                raise OSError('VL53L1X の起動待ちがタイムアウトしました')
            time.sleep(0.005)
        self.write(0x002D, DEFAULT_CONFIGURATION)
        self.start()
        deadline = time.monotonic() + 1.0
        while not self.data_ready():
            if time.monotonic() > deadline:
                raise OSError('VL53L1X の初回測定がタイムアウトしました')
            time.sleep(0.005)
        self.clear_interrupt()
        self.stop()
        self.write_u8(REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09)
        self.write_u8(0x000B, 0x00)
        # 短距離モード（床まで約 10 cm なので十分。外光の影響を受けにくい）
        self.write_u8(REG_PHASECAL_CONFIG_TIMEOUT_MACROP, 0x14)
        self.write_u8(REG_RANGE_CONFIG_VCSEL_PERIOD_A, 0x07)
        self.write_u8(REG_RANGE_CONFIG_VCSEL_PERIOD_B, 0x05)
        self.write_u8(REG_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38)
        self.write(REG_SD_CONFIG_WOI_SD0, b'\x07\x05')
        self.write(REG_SD_CONFIG_INITIAL_PHASE_SD0, b'\x06\x06')
        a_hi, b_hi = TIMING_BUDGET_SHORT[timing_budget_ms]
        self.write_u16(REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, a_hi)
        self.write_u16(REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, b_hi)
        clock_pll = self.read_u16(REG_RESULT_OSC_CALIBRATE_VAL) & 0x03FF
        period = int(clock_pll * intermeasurement_ms * 1.075)
        self.write(REG_SYSTEM_INTERMEASUREMENT_PERIOD, period.to_bytes(4, 'big'))
        self.start()

    def start(self):
        self.write_u8(REG_SYSTEM_MODE_START, 0x40)

    def stop(self):
        self.write_u8(REG_SYSTEM_MODE_START, 0x00)

    def clear_interrupt(self):
        self.write_u8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)

    def data_ready(self) -> bool:
        # GPIO_HV_MUX__CTRL = 0x01（アクティブ High）なので bit0 = 1 で新しい測定値あり
        return bool(self.read_u8(REG_GPIO_TIO_HV_STATUS) & 0x01)

    def read_range(self):
        """(RANGE_STATUS の生の値, 距離 mm) を返し、割り込みを解除する。"""
        status = self.read_u8(REG_RESULT_RANGE_STATUS) & 0x1F
        distance = self.read_u16(REG_RESULT_DISTANCE_MM)
        self.clear_interrupt()
        return status, distance


class CliffSensor:
    """1 個のセンサーの接続状態と段差判定。"""

    def __init__(self, name: str, xshut_gpio: int, address: int):
        self.name = name
        self.xshut_gpio = xshut_gpio
        self.address = address
        self.line = None
        self.device = None
        self.next_retry = 0.0
        self.error = '未接続'
        self.distance_mm = None
        self.range_status = None
        self.last_sample = 0.0
        self.cliff = False
        self.far_count = 0
        self.floor_count = 0

    def reset_state(self):
        self.distance_mm = None
        self.range_status = None
        self.last_sample = 0.0
        self.cliff = False
        self.far_count = 0
        self.floor_count = 0


class CliffSensorNode(Node):
    """前後の VL53L1X を読み、段差の有無を /cliff_status（JSON）に publish する。"""

    def __init__(self):
        super().__init__('cliff_sensor_node')
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('gpio_chip', '')
        self.declare_parameter('names', ['front', 'rear'])
        self.declare_parameter('xshut_gpios', [17, 27])
        self.declare_parameter('addresses', [0x2A, 0x2B])
        self.declare_parameter('topic', '/cliff_status')
        self.declare_parameter('poll_rate', 100.0)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('timing_budget_ms', 20)
        self.declare_parameter('intermeasurement_ms', 25)
        self.declare_parameter('cliff_distance_mm', 130)
        self.declare_parameter('cliff_confirm', 2)
        self.declare_parameter('clear_confirm', 5)
        self.declare_parameter('sample_timeout', 0.3)

        self.i2c_bus = int(self.get_parameter('i2c_bus').value)
        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        names = list(self.get_parameter('names').value)
        gpios = [int(v) for v in self.get_parameter('xshut_gpios').value]
        addresses = [int(v) for v in self.get_parameter('addresses').value]
        if not (len(names) == len(gpios) == len(addresses)):
            raise ValueError('names / xshut_gpios / addresses の個数をそろえてください')
        if DEFAULT_ADDRESS in addresses:
            raise ValueError('addresses に 0x29 は使えません（BNO055 と重なる）')
        self.timing_budget_ms = int(self.get_parameter('timing_budget_ms').value)
        if self.timing_budget_ms not in TIMING_BUDGET_SHORT:
            raise ValueError(
                f'timing_budget_ms は {sorted(TIMING_BUDGET_SHORT)} のどれかにしてください')
        self.intermeasurement_ms = max(
            int(self.get_parameter('intermeasurement_ms').value), self.timing_budget_ms)
        self.cliff_distance_mm = int(self.get_parameter('cliff_distance_mm').value)
        self.cliff_confirm = max(int(self.get_parameter('cliff_confirm').value), 1)
        self.clear_confirm = max(int(self.get_parameter('clear_confirm').value), 1)
        self.sample_timeout = float(self.get_parameter('sample_timeout').value)

        self.sensors = [CliffSensor(n, g, a) for n, g, a in zip(names, gpios, addresses)]
        self.pub = self.create_publisher(String, str(self.get_parameter('topic').value), 10)
        self._chip = None
        self._last_error_log = {}
        self._gpio_ready = self._setup_gpio()

        poll_rate = float(self.get_parameter('poll_rate').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(poll_rate, 1.0), self._poll_cb)
        self.create_timer(1.0 / max(publish_rate, 1.0), self._publish)

    # --- GPIO（XSHUT） ---
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

    def _setup_gpio(self) -> bool:
        if gpiod is None:
            for sensor in self.sensors:
                sensor.error = 'python3-libgpiod がありません'
            self.get_logger().error('python3-libgpiod がないため落下防止センサーを使えません')
            return False
        try:
            self._chip = self._find_chip()
            for sensor in self.sensors:
                sensor.line = self._chip.get_line(sensor.xshut_gpio)
                sensor.line.request(consumer='cliff_sensor_node',
                                    type=gpiod.LINE_REQ_DIR_OUT, default_val=0)
        except OSError as e:
            for sensor in self.sensors:
                sensor.error = f'XSHUT の GPIO を使えません: {e}'
            self.get_logger().error(f'XSHUT の GPIO を使えません: {e}')
            return False
        time.sleep(0.01)
        return True

    # --- 接続 ---
    def _connect(self, sensor: CliffSensor):
        sensor.line.set_value(0)
        time.sleep(0.01)
        sensor.line.set_value(1)
        # 0x29 は BNO055 と共有なので、起動待ちは読まずに時間で待つ（tBOOT 最大 1.2 ms）
        time.sleep(0.01)
        boot = VL53L1X(self.i2c_bus, DEFAULT_ADDRESS)
        try:
            boot.write_u8(REG_I2C_SLAVE_DEVICE_ADDRESS, sensor.address)
        finally:
            boot.close()
        time.sleep(0.002)
        device = VL53L1X(self.i2c_bus, sensor.address)
        try:
            device.initialize(self.timing_budget_ms, self.intermeasurement_ms)
        except OSError:
            device.close()
            raise
        sensor.device = device
        sensor.error = ''
        sensor.reset_state()
        self.get_logger().info(
            f'VL53L1X（{sensor.name}）接続: /dev/i2c-{self.i2c_bus}, 0x{sensor.address:02x}, '
            f'XSHUT GPIO{sensor.xshut_gpio}')

    def _disconnect(self, sensor: CliffSensor, error: str):
        if sensor.device is not None:
            sensor.device.close()
        sensor.device = None
        sensor.error = error
        sensor.reset_state()
        if sensor.line is not None:
            try:
                sensor.line.set_value(0)
            except OSError:
                pass
        sensor.next_retry = time.monotonic() + 2.0

    def _warn(self, sensor: CliffSensor, message: str):
        now = time.monotonic()
        if now - self._last_error_log.get(sensor.name, 0.0) >= 10.0:
            self.get_logger().warn(message)
            self._last_error_log[sensor.name] = now

    # --- 測定 ---
    def _update(self, sensor: CliffSensor, status: int, distance: int):
        sensor.range_status = status
        sensor.distance_mm = distance
        sensor.last_sample = time.monotonic()
        if status == RANGE_VALID:
            far = distance > self.cliff_distance_mm
        else:
            # 最短距離未満は床（か物）が近いだけ。それ以外の無効値は反射が返らないので段差側に倒す
            far = status != RANGE_MIN_RANGE_FAIL
        if far:
            sensor.far_count += 1
            sensor.floor_count = 0
        else:
            sensor.floor_count += 1
            sensor.far_count = 0
        before = sensor.cliff
        if not sensor.cliff and sensor.far_count >= self.cliff_confirm:
            sensor.cliff = True
        elif sensor.cliff and sensor.floor_count >= self.clear_confirm:
            sensor.cliff = False
        if sensor.cliff != before:
            text = '段差を検出' if sensor.cliff else '床を検出（解除）'
            self.get_logger().info(
                f'落下防止センサー（{sensor.name}）: {text} '
                f'距離 {distance} mm / 状態 {status}')
            return True
        return False

    def _poll_cb(self):
        if not self._gpio_ready:
            return
        now = time.monotonic()
        changed = False
        for sensor in self.sensors:
            if sensor.device is None:
                if now < sensor.next_retry:
                    continue
                try:
                    self._connect(sensor)
                except OSError as e:
                    self._warn(sensor, f'VL53L1X（{sensor.name}）に接続できません: {e}')
                    self._disconnect(sensor, f'接続できません: {e}')
                    changed = True
                continue
            try:
                if sensor.device.data_ready():
                    status, distance = sensor.device.read_range()
                    changed |= self._update(sensor, status, distance)
                elif sensor.last_sample and now - sensor.last_sample > self.sample_timeout:
                    raise OSError('測定値が更新されません')
            except OSError as e:
                self._warn(sensor, f'VL53L1X（{sensor.name}）読み取りエラー: {e}')
                self._disconnect(sensor, f'読み取りエラー: {e}')
                changed = True
        if changed:
            self._publish()

    def _publish(self):
        now = time.monotonic()
        sensors = {}
        for sensor in self.sensors:
            ok = sensor.device is not None and bool(sensor.last_sample) and (
                now - sensor.last_sample <= self.sample_timeout)
            sensors[sensor.name] = {
                'ok': ok,
                'cliff': sensor.cliff if ok else None,
                'distance_mm': sensor.distance_mm,
                'range_status': sensor.range_status,
                'address': f'0x{sensor.address:02x}',
                'error': sensor.error,
            }
        msg = String()
        msg.data = json.dumps({
            'stamp': round(time.time(), 3),
            'cliff_distance_mm': self.cliff_distance_mm,
            'sensors': sensors,
        }, ensure_ascii=False)
        self.pub.publish(msg)

    def destroy_node(self):
        for sensor in self.sensors:
            if sensor.device is not None:
                try:
                    sensor.device.stop()
                except OSError:
                    pass
                sensor.device.close()
            if sensor.line is not None:
                try:
                    sensor.line.set_value(0)
                    sensor.line.release()
                except OSError:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CliffSensorNode()
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
