#!/usr/bin/env python3
"""Yahboom CUBE ケースの横の OLED（SSD1306 128x32、I2C 7 番 0x3C）に Jetson の状態を出す。

標準ライブラリと Pillow（JetPack の /usr/bin/python3 に入っている）だけで動く。
"""
import fcntl
import os
import signal
import subprocess
import sys
import time

from PIL import Image, ImageDraw, ImageFont

I2C_BUS = int(os.environ.get('OLED_I2C_BUS', '7'))
I2C_ADDR = int(os.environ.get('OLED_I2C_ADDR', '0x3c'), 16)
WIDTH, HEIGHT = 128, 32
I2C_SLAVE = 0x0703
INIT = [0xAE, 0xD5, 0x80, 0xA8, HEIGHT - 1, 0xD3, 0x00, 0x40, 0x8D, 0x14, 0x20, 0x00,
        0xA1, 0xC8, 0xDA, 0x02, 0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF]
IP_SKIP = ('lo', 'docker', 'l4tbr', 'veth', 'br-')


class Oled:
    def __init__(self, bus, addr):
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, addr)
        self.command(*INIT)

    def command(self, *cmds):
        for c in cmds:
            os.write(self.fd, bytes([0x00, c]))

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
        self.command(0x21, 0, WIDTH - 1, 0x22, 0, HEIGHT // 8 - 1)
        for i in range(0, len(data), 16):
            os.write(self.fd, bytes([0x40]) + data[i:i + 16])

    def off(self):
        self.show(Image.new('1', (WIDTH, HEIGHT)))
        self.command(0xAE)


def cpu_times():
    with open('/proc/stat') as f:
        v = [int(x) for x in f.readline().split()[1:]]
    return sum(v), v[3] + v[4]


def cpu_temp():
    base = '/sys/devices/virtual/thermal'
    for z in sorted(os.listdir(base)):
        try:
            with open(f'{base}/{z}/type') as f:
                if f.read().strip() != 'cpu-thermal':
                    continue
            with open(f'{base}/{z}/temp') as f:
                return int(f.read()) / 1000.0
        except OSError:
            continue
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
        if len(parts) > 3 and not parts[1].startswith(IP_SKIP):
            addrs.append(parts[3].split('/')[0])
    addrs.sort(key=lambda a: not a.startswith('100.'))
    return addrs


def main():
    oled = Oled(I2C_BUS, I2C_ADDR)

    def stop(*_):
        oled.off()
        sys.exit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    font = ImageFont.load_default()
    last = cpu_times()
    ips, ips_at, tick = [], 0.0, 0
    while True:
        time.sleep(1.0)
        now = cpu_times()
        dt, di = now[0] - last[0], now[1] - last[1]
        last = now
        cpu = 100.0 * (dt - di) / dt if dt else 0.0
        if time.monotonic() - ips_at > 30:
            ips, ips_at = ip_addrs(), time.monotonic()
        temp = cpu_temp()
        mu, mt = mem_gib()
        du, dtot = disk_gb()
        ip = ips[(tick // 5) % len(ips)] if ips else 'no network'
        tick += 1
        lines = [
            f'CPU:{cpu:3.0f}%  ' + (f'{temp:.1f}C' if temp is not None else '-'),
            f'RAM:{mu:.1f}/{mt:.1f}G',
            f'SSD:{du:.0f}/{dtot:.0f}G',
            f'IP:{ip}',
        ]
        image = Image.new('1', (WIDTH, HEIGHT))
        draw = ImageDraw.Draw(image)
        for i, text in enumerate(lines):
            draw.text((0, -2 + i * 8), text, font=font, fill=255)
        try:
            oled.show(image)
        except OSError as e:
            print(f'OLED に書けません: {e}', file=sys.stderr, flush=True)
            time.sleep(5)
            try:
                oled = Oled(I2C_BUS, I2C_ADDR)
            except OSError:
                pass


if __name__ == '__main__':
    main()
