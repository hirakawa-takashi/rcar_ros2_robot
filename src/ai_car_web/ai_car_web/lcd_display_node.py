#!/usr/bin/env python3
"""ZJY-IPS130-V2.0（ST7789 1.3 インチ 240x240 IPS 液晶、SPI）へロボット状態を表示するノード。

seg_display_node（TM1637 7 セグ）の置き換え。運転モードを大きく表示し、
下段に障害物状態・前方距離・入力電圧・ゲームパッド接続・IP アドレスを表示する。
"""

import json
import socket
import time

import numpy as np

try:
    import gpiod
except ImportError:
    gpiod = None

try:
    import spidev
except ImportError:
    spidev = None

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = ImageDraw = ImageFont = None

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

WIDTH = 240
HEIGHT = 240

FONT_CANDIDATES = [
    '/usr/share/fonts/opentype/ipafont-gothic/ipagp.ttf',
    '/usr/share/fonts/truetype/fonts-japanese-gothic.ttf',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
]

# 表示コード → (見出し, 背景色)。seg_display_node の /display_state と同じコード体系
STATES = {
    'boot': ('起動中', (40, 40, 40)),
    'Err ': ('システム未受信', (120, 20, 20)),
    'LoU ': ('低電圧', (140, 60, 0)),
    'ObSt': ('障害物停止', (150, 20, 20)),
    'SLo ': ('減速', (140, 100, 0)),
    'StoP': ('停止', (110, 20, 20)),
    'AUto': ('自動', (20, 60, 130)),
    'HAnd': ('手動', (20, 100, 50)),
    'rdy ': ('待機', (30, 70, 40)),
}


class ST7789Display:
    """spidev + libgpiod v1 API で ST7789（CS なし 7 ピン版）を駆動する。"""

    def __init__(self, chip, bus: int, device: int, speed_hz: int,
                 dc_gpio: int, rst_gpio: int, bl_gpio: int, rotation: int):
        self.chip = chip
        self.dc = chip.get_line(dc_gpio)
        self.rst = chip.get_line(rst_gpio)
        self.bl = chip.get_line(bl_gpio) if bl_gpio >= 0 else None
        self.spi = None
        try:
            self.dc.request(consumer='lcd_display_node',
                            type=gpiod.LINE_REQ_DIR_OUT)
            self.rst.request(consumer='lcd_display_node',
                             type=gpiod.LINE_REQ_DIR_OUT)
            if self.bl is not None:
                self.bl.request(consumer='lcd_display_node',
                                type=gpiod.LINE_REQ_DIR_OUT)
            self.spi = spidev.SpiDev()
            self.spi.open(bus, device)
            self.spi.max_speed_hz = speed_hz
            # CS 線なしモジュールは SPI モード 3（CPOL=1, CPHA=1）で安定する
            self.spi.mode = 0b11
            self._init_panel(rotation)
        except Exception:
            self.close()
            raise

    def _cmd(self, cmd: int, data=None):
        self.dc.set_value(0)
        self.spi.writebytes([cmd])
        if data:
            self.dc.set_value(1)
            self.spi.writebytes(list(data))

    def _init_panel(self, rotation: int):
        self.rst.set_value(1)
        time.sleep(0.01)
        self.rst.set_value(0)
        time.sleep(0.01)
        self.rst.set_value(1)
        time.sleep(0.12)
        self._cmd(0x01)            # SWRESET
        time.sleep(0.15)
        self._cmd(0x11)            # SLPOUT
        time.sleep(0.12)
        self._cmd(0x3A, [0x55])    # COLMOD: 16bit RGB565
        madctl = {0: 0x00, 90: 0x60, 180: 0xC0, 270: 0xA0}.get(rotation, 0x00)
        self._cmd(0x36, [madctl])  # MADCTL
        self._cmd(0x21)            # INVON（IPS パネルは反転が正）
        self._cmd(0x13)            # NORON
        self._cmd(0x29)            # DISPON
        time.sleep(0.05)
        if self.bl is not None:
            self.bl.set_value(1)

    def _set_window(self):
        self._cmd(0x2A, [0, 0, 0, WIDTH - 1])
        self._cmd(0x2B, [0, 0, 0, HEIGHT - 1])
        self._cmd(0x2C)

    def show(self, image):
        rgb = np.asarray(image.convert('RGB'), dtype=np.uint16)
        v = ((rgb[:, :, 0] & 0xF8) << 8) | ((rgb[:, :, 1] & 0xFC) << 3) | (rgb[:, :, 2] >> 3)
        buf = v.astype('>u2').tobytes()
        self._set_window()
        self.dc.set_value(1)
        for k in range(0, len(buf), 4096):
            self.spi.writebytes2(buf[k:k + 4096])

    def close(self):
        if self.spi is not None:
            try:
                self.spi.close()
            except Exception:
                pass
            self.spi = None
        for line in (self.bl, self.dc, self.rst):
            if line is not None:
                try:
                    if line is self.bl:
                        line.set_value(0)
                    line.release()
                except Exception:
                    pass
        self.bl = self.dc = self.rst = None
        if self.chip is not None and hasattr(self.chip, 'close'):
            try:
                self.chip.close()
            except Exception:
                pass
        self.chip = None


def _local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except OSError:
        return '-'


class LcdDisplayNode(Node):
    """各種トピックからロボット状態を判定して ST7789 液晶へ描画する。"""

    def __init__(self):
        super().__init__('lcd_display_node')
        self.declare_parameter('gpio_chip', '')
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', 32000000)
        self.declare_parameter('dc_gpio', 25)
        self.declare_parameter('rst_gpio', 24)
        self.declare_parameter('bl_gpio', 23)
        self.declare_parameter('rotation', 0)
        self.declare_parameter('update_rate', 4.0)
        self.declare_parameter('font_path', '')
        self.declare_parameter('system_status_topic', '/system_status')
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('drive_mode_topic', '/drive_mode')
        self.declare_parameter('autonomy_status_topic', '/autonomy_status')
        self.declare_parameter('state_topic', '/display_state')
        self.declare_parameter('status_timeout', 3.0)

        self.gpio_chip = str(self.get_parameter('gpio_chip').value)
        self.spi_bus = int(self.get_parameter('spi_bus').value)
        self.spi_device = int(self.get_parameter('spi_device').value)
        self.spi_speed = int(self.get_parameter('spi_speed_hz').value)
        self.dc_gpio = int(self.get_parameter('dc_gpio').value)
        self.rst_gpio = int(self.get_parameter('rst_gpio').value)
        self.bl_gpio = int(self.get_parameter('bl_gpio').value)
        self.rotation = int(self.get_parameter('rotation').value)
        self.status_timeout = float(self.get_parameter('status_timeout').value)
        self._display = None
        self._next_display_retry = 0.0
        self._last_display_error = 0.0
        self._missing_dep_logged = False
        self._start_time = time.monotonic()
        self._system = None
        self._system_stamp = None
        self._obstacle = None
        self._obstacle_stamp = None
        self._last_motion = None
        self._drive_mode = None
        self._drive_mode_stamp = None
        self._autonomy = None
        self._autonomy_stamp = None
        self._state = None
        self._last_frame_key = None
        self._ip = '-'
        self._ip_stamp = 0.0
        self._fonts = self._load_fonts(str(self.get_parameter('font_path').value))

        tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, tl)
        self.create_subscription(
            String, self.get_parameter('system_status_topic').value,
            self._system_cb, 10)
        self.create_subscription(
            String, self.get_parameter('obstacle_topic').value,
            self._obstacle_cb, 10)
        self.create_subscription(
            Twist, self.get_parameter('joy_cmd_topic').value,
            self._motion_cb, 10)
        self.create_subscription(
            Twist, self.get_parameter('cmd_vel_topic').value,
            self._motion_cb, 10)
        self.create_subscription(
            String, self.get_parameter('drive_mode_topic').value,
            self._drive_mode_cb, tl)
        self.create_subscription(
            String, self.get_parameter('autonomy_status_topic').value,
            self._autonomy_cb, 10)
        rate = float(self.get_parameter('update_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._timer_cb)

    # ---------- フォント ----------
    def _load_fonts(self, font_path: str):
        if ImageFont is None:
            return None
        paths = [font_path] if font_path else []
        paths += FONT_CANDIDATES
        for path in paths:
            try:
                return {
                    'big': ImageFont.truetype(path, 64),
                    'mid': ImageFont.truetype(path, 24),
                    'small': ImageFont.truetype(path, 18),
                }
            except OSError:
                continue
        self.get_logger().warn('TrueType フォントが見つからないため既定フォントで描画します')
        f = ImageFont.load_default()
        return {'big': f, 'mid': f, 'small': f}

    # ---------- コールバック ----------
    @staticmethod
    def _parse(msg: String):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return None
        return payload if isinstance(payload, dict) else None

    def _system_cb(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            self._system = payload
            self._system_stamp = time.monotonic()

    def _obstacle_cb(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            self._obstacle = payload
            self._obstacle_stamp = time.monotonic()

    def _drive_mode_cb(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            self._drive_mode = payload
            self._drive_mode_stamp = time.monotonic()

    def _autonomy_cb(self, msg: String):
        payload = self._parse(msg)
        if payload is not None:
            self._autonomy = payload
            self._autonomy_stamp = time.monotonic()

    def _motion_cb(self, msg: Twist):
        if (abs(msg.linear.x) > 0.05 or abs(msg.linear.y) > 0.05
                or abs(msg.angular.z) > 0.05):
            self._last_motion = time.monotonic()

    # ---------- 表示デバイス ----------
    def _warn_display_error(self, message: str, error):
        now = time.monotonic()
        if now - self._last_display_error >= 5.0:
            self.get_logger().warn(f'{message}: {error}')
            self._last_display_error = now

    def _find_chip(self):
        if self.gpio_chip:
            return gpiod.Chip(self.gpio_chip)
        for index in range(10):
            path = f'/dev/gpiochip{index}'
            try:
                chip = gpiod.Chip(path)
            except OSError:
                continue
            if chip.label() == 'pinctrl-rp1':
                return chip
            if hasattr(chip, 'close'):
                chip.close()
        raise OSError('pinctrl-rp1 の gpiochip が見つかりません')

    def _ensure_display(self):
        missing = [name for name, mod in (
            ('python3-libgpiod', gpiod), ('python3-spidev', spidev),
            ('python3-pil', Image)) if mod is None]
        if missing:
            if not self._missing_dep_logged:
                self.get_logger().warn(
                    f'{" / ".join(missing)} がないため液晶表示を無効化します')
                self._missing_dep_logged = True
            return False
        if self._display is not None:
            return True
        now = time.monotonic()
        if now < self._next_display_retry:
            return False
        try:
            chip = self._find_chip()
            self._display = ST7789Display(
                chip, self.spi_bus, self.spi_device, self.spi_speed,
                self.dc_gpio, self.rst_gpio, self.bl_gpio, self.rotation)
            self._last_frame_key = None
            self.get_logger().info('ST7789 液晶表示を接続しました')
        except Exception as e:
            self._display = None
            self._warn_display_error('ST7789 液晶表示に接続できません', e)
            self._next_display_retry = now + 2.0
            return False
        return True

    def _close_display(self):
        if self._display is not None:
            self._display.close()
            self._display = None

    # ---------- 状態判定（seg_display_node と同じ優先順位） ----------
    def _fresh(self, stamp):
        return stamp is not None and time.monotonic() - stamp < self.status_timeout

    def _state_code(self, now: float) -> str:
        if not self._fresh(self._system_stamp):
            if self._system_stamp is None and now - self._start_time < 30.0:
                return 'boot'
            return 'Err '
        power = self._system.get('power') or {}
        throttled = power.get('throttled') or {}
        if throttled.get('under_voltage_now', False):
            return 'LoU '
        if self._fresh(self._obstacle_stamp):
            level = self._obstacle.get('level')
            if level == 'stop':
                return 'ObSt'
            if level == 'slow':
                return 'SLo '
        if self._fresh(self._drive_mode_stamp):
            mode = self._drive_mode.get('mode')
            if mode == 'stop':
                return 'StoP'
            if mode == 'auto':
                return 'AUto'
        if self._last_motion is not None and now - self._last_motion < 1.0:
            return 'HAnd'
        return 'rdy '

    # ---------- 描画 ----------
    def _frame_data(self, code: str):
        power = (self._system or {}).get('power') or {}
        cpu = (self._system or {}).get('cpu') or {}
        obstacle = self._obstacle if self._fresh(self._obstacle_stamp) else {}
        mode = self._drive_mode if self._fresh(self._drive_mode_stamp) else {}
        auto = self._autonomy if self._fresh(self._autonomy_stamp) else {}
        inputs = mode.get('inputs') or {}
        front = obstacle.get('front_distance')
        volt = power.get('input_volt')
        temp = cpu.get('temperature_c')
        return {
            'code': code,
            'mode_label': mode.get('label') or '-',
            'behavior': auto.get('behavior_label') if mode.get('mode') == 'auto' else None,
            'obstacle': obstacle.get('level') or '-',
            'front': f'{front:.2f} m' if isinstance(front, (int, float)) else '-',
            'volt': f'{volt:.2f} V' if isinstance(volt, (int, float)) else '-',
            'temp': f'{temp:.0f}°C' if isinstance(temp, (int, float)) else '-',
            'joy_text': 'PAD 接続' if inputs.get('joy_connected') else 'PAD なし',
            'ip': self._ip,
        }

    def _render(self, d) -> 'Image.Image':
        title, bg = STATES.get(d['code'], ('?', (40, 40, 40)))
        img = Image.new('RGB', (WIDTH, HEIGHT), (0, 0, 0))
        draw = ImageDraw.Draw(img)
        f = self._fonts
        # 上段: 状態見出し（背景色つき）
        draw.rectangle([0, 0, WIDTH, 110], fill=bg)
        size = 'big' if len(title) <= 3 else 'mid'
        box = draw.textbbox((0, 0), title, font=f[size])
        tw, th = box[2] - box[0], box[3] - box[1]
        draw.text(((WIDTH - tw) / 2 - box[0], (110 - th) / 2 - box[1]),
                  title, font=f[size], fill=(255, 255, 255))
        # 中段: 運転モードと自律行動
        line = f'モード: {d["mode_label"]}'
        if d['behavior']:
            line += f' / {d["behavior"]}'
        draw.text((8, 118), line, font=f['small'], fill=(200, 230, 255))
        # 下段: センサーとシステム
        rows = [
            (f'前方 {d["front"]}', f'障害物 {d["obstacle"]}'),
            (f'入力 {d["volt"]}', f'CPU {d["temp"]}'),
            (d['joy_text'], ''),
        ]
        y = 146
        for left, right in rows:
            draw.text((8, y), left, font=f['small'], fill=(230, 230, 230))
            if right:
                draw.text((124, y), right, font=f['small'], fill=(230, 230, 230))
            y += 24
        draw.rectangle([0, HEIGHT - 22, WIDTH, HEIGHT], fill=(25, 25, 25))
        draw.text((8, HEIGHT - 21), d['ip'], font=f['small'], fill=(150, 200, 150))
        return img

    def _timer_cb(self):
        now = time.monotonic()
        code = self._state_code(now)
        if code != self._state:
            self._state = code
            msg = String()
            msg.data = code
            self._state_pub.publish(msg)
            self.get_logger().info(f'表示状態: {code}')

        if now - self._ip_stamp > 30.0:
            self._ip = _local_ip()
            self._ip_stamp = now

        if not self._ensure_display():
            return
        data = self._frame_data(code)
        key = json.dumps(data, sort_keys=True, ensure_ascii=False)
        if key == self._last_frame_key:
            return
        try:
            self._display.show(self._render(data))
            self._last_frame_key = key
        except Exception as e:
            self._warn_display_error('ST7789 表示エラー', e)
            self._close_display()
            self._next_display_retry = time.monotonic() + 2.0

    def destroy_node(self):
        if self._display is not None:
            try:
                self._display.show(Image.new('RGB', (WIDTH, HEIGHT), (0, 0, 0)))
            except Exception:
                pass
        self._close_display()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LcdDisplayNode()
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
