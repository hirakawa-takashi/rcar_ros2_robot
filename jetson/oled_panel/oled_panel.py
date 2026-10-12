#!/usr/bin/env python3
"""Yahboom CUBE ケースの横の OLED（SSD1306 128x32、I2C 7 番 0x3C）に Jetson の状態を出す。

大きな文字の 2 行で、CPU/GPU → メモリ/SSD → IP アドレスの画面を 3 秒ごとに切り替える。
ケースの RGB の光（同じ I2C の 0x0E）は、AI の負荷（GPU の負荷率の 5 秒平均）で呼吸の色と速さを変える。
CPU が熱いときは負荷にかかわらず赤の速い呼吸にする。
標準ライブラリと Pillow（JetPack の /usr/bin/python3 に入っている）だけで動く。
"""
import fcntl
import os
import signal
import subprocess
import sys
import time
from collections import deque

from PIL import Image, ImageDraw, ImageFont

I2C_BUS = int(os.environ.get('OLED_I2C_BUS', '7'))
OLED_ADDR = 0x3C
CASE_ADDR = 0x0E
WIDTH, HEIGHT = 128, 32
I2C_SLAVE = 0x0703
INIT = [0xAE, 0xD5, 0x80, 0xA8, HEIGHT - 1, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
        0xA1, 0xC8, 0xDA, 0x02, 0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF]
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
FONT_SIZE = 14
PAGE_SEC = 3
HOT_C = float(os.environ.get('OLED_HOT_C', '70'))
RGB_EFFECT, RGB_SPEED, RGB_COLOR = 0x04, 0x05, 0x06
BREATHING, RED = 1, 0
# (GPU の負荷率の下限 %, 色, 速さ): 色 0 赤・1 緑・2 青・3 黄、速さ 1 低速〜3 高速
LOAD_LEVELS = ((70, RED, 3), (40, 3, 2), (10, 1, 2), (0, 2, 1))
LOAD_AVG_SEC = 5
GPU_LOAD = '/sys/devices/platform/bus@0/17000000.gpu/load'
IF_NAMES = (('tailscale', 'Tailscale'), ('wl', 'Wi-Fi'), ('en', 'LAN'), ('eth', 'LAN'))


class I2cDevice:
    def __init__(self, bus, addr):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)

    def write(self, data):
        os.write(self.fd, bytes(data))


class Oled(I2cDevice):
    def __init__(self, bus):
        super().__init__(bus, OLED_ADDR)
        for c in INIT:
            self.write([0x00, c])

    def show(self, image):
        data = bytearray(WIDTH * HEIGHT // 8)
        px = image.load()
        for page in range(HEIGHT // 8):
            for x in range(WIDTH):
                b = 0
                for bit in range(8):
                    if px[x, page * 8 + bit]:
                        b |= 1 << bit
                data[page * WIDTH + x] = b
        for c in (0x21, 0, WIDTH - 1, 0x22, 0, HEIGHT // 8 - 1):
            self.write([0x00, c])
        for i in range(0, len(data), 16):
            self.write(bytes([0x40]) + data[i:i + 16])

    def off(self):
        self.show(Image.new('1', (WIDTH, HEIGHT)))
        self.write([0x00, 0xAE])


class CaseRgb(I2cDevice):
    def __init__(self, bus):
        super().__init__(bus, CASE_ADDR)
        self.state = None

    def update(self, color, speed):
        if (color, speed) == self.state:
            return
        self.write([RGB_COLOR, color])
        self.write([RGB_SPEED, speed])
        self.write([RGB_EFFECT, BREATHING])
        self.state = (color, speed)
        print(f'RGB: 色 {color}・速さ {speed}', flush=True)


def light_for(gpu_avg, cpu_c):
    if cpu_c is not None and cpu_c >= HOT_C:
        return RED, 3
    for low, color, speed in LOAD_LEVELS:
        if gpu_avg >= low:
            return color, speed
    return LOAD_LEVELS[-1][1:]


def cpu_times():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + v[4]


def thermal(name):
    base = '/sys/devices/virtual/thermal'
    for z in sorted(os.listdir(base)):
        try:
            with open(f'{base}/{z}/type') as f:
                if f.read().strip() != name:
                    continue
            with open(f'{base}/{z}/temp') as f:
                return int(f.read()) / 1000.0
        except OSError:
            continue
    return None


def gpu_load():
    try:
        with open(GPU_LOAD) as f:
            return int(f.read()) / 10.0
    except (OSError, ValueError):
        return None


def mem_gib():
    info = {}
    with open('/proc/meminfo') as f:
        for line in f:
            k, v = line.split(':', 1)
            info[k] = int(v.split()[0])
    total = info['MemTotal'] / 1048576
    return total - info['MemAvailable'] / 1048576, total


def disk_gb():
    st = os.statvfs('/')
    total = st.f_blocks * st.f_frsize / 1e9
    return total - st.f_bavail * st.f_frsize / 1e9, total


def ip_addrs():
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr', 'show'], capture_output=True,
                             text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    addrs = []
    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        for prefix, label in IF_NAMES:
            if parts[1].startswith(prefix):
                addrs.append((label, parts[3].split('/')[0]))
                break
    return addrs


def pct(v):
    return f'{v:.0f}%' if v is not None else '-'


def deg(v):
    return f'{v:.0f}°C' if v is not None else '-'


def pages(cpu, cpu_c, gpu, gpu_c, mem, disk, ips):
    yield (('CPU', pct(cpu), deg(cpu_c)), ('GPU', pct(gpu), deg(gpu_c)))
    yield (('RAM', f'{mem[0]:.1f}/{mem[1]:.1f}G', ''), ('SSD', f'{disk[0]:.0f}/{disk[1]:.0f}G', ''))
    for label, ip in ips or [('IP', 'no network')]:
        yield ((label, '', ''), ('', ip, ''))


def render(page, font):
    image = Image.new('1', (WIDTH, HEIGHT))
    draw = ImageDraw.Draw(image)
    for row, (label, value, extra) in enumerate(page):
        y = row * 16
        if label and not value:
            draw.text((0, y), label, font=font, fill=255)
            continue
        x = 0
        if label:
            draw.text((0, y), label, font=font, fill=255)
            x = 38
        if extra:
            draw.text((x + 50 - draw.textlength(value, font=font), y), value, font=font, fill=255)
            draw.text((WIDTH - draw.textlength(extra, font=font), y), extra, font=font, fill=255)
        else:
            draw.text((max(x, WIDTH - draw.textlength(value, font=font)), y), value,
                      font=font, fill=255)
    return image


def main():
    oled = Oled(I2C_BUS)
    try:
        rgb = CaseRgb(I2C_BUS)
    except OSError as e:
        print(f'ケースの RGB を開けません: {e}', file=sys.stderr, flush=True)
        rgb = None

    def stop(*_):
        oled.off()
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        font = ImageFont.truetype(FONT, FONT_SIZE)
    except OSError:
        font = ImageFont.load_default()
    last = cpu_times()
    ips, ips_at, tick = [], 0.0, 0
    gpu_hist = deque(maxlen=LOAD_AVG_SEC)
    while True:
        time.sleep(1.0)
        now = cpu_times()
        dt, di = now[0] - last[0], now[1] - last[1]
        last = now
        cpu = 100.0 * (dt - di) / dt if dt else 0.0
        if time.monotonic() - ips_at > 30:
            ips, ips_at = ip_addrs(), time.monotonic()
        cpu_c = thermal('cpu-thermal')
        gpu = gpu_load()
        if gpu is not None:
            gpu_hist.append(gpu)
        gpu_avg = sum(gpu_hist) / len(gpu_hist) if gpu_hist else 0.0
        all_pages = list(pages(cpu, cpu_c, gpu, thermal('gpu-thermal'),
                               mem_gib(), disk_gb(), ips))
        page = all_pages[(tick // PAGE_SEC) % len(all_pages)]
        tick += 1
        try:
            if rgb is not None:
                rgb.update(*light_for(gpu_avg, cpu_c))
            oled.show(render(page, font))
        except OSError as e:
            print(f'I2C に書けません: {e}', file=sys.stderr, flush=True)
            time.sleep(5)
            try:
                oled = Oled(I2C_BUS)
                if rgb is not None:
                    rgb.state = None
            except OSError:
                pass


if __name__ == '__main__':
    main()
