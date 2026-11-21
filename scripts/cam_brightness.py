#!/usr/bin/env python3
"""カメラ映像の明るさを /camera/image_raw/compressed から測る見張り。

ダッシュボードの映像が「真っ黒」と疑ったときの切り分けに使う。
黒い原因が「部屋が暗い」なら自動露出が上限まで利いていて、平均輝度は 2 前後、
色は青に振れ、20 枚読み続けても数値は動かない。露出が止まっている（故障）と
同じく低い値になることがあるので、部屋の電気をつけて測り直し、
平均輝度が 100 を超えるかを基準に判断する。

使い方（ラズパイ上）:
    source /opt/ros/jazzy/setup.bash
    python3 scripts/cam_brightness.py            # 20 枚
    python3 scripts/cam_brightness.py --frames 40 --save /tmp/frame.jpg
"""

import argparse
import io
import sys

import rclpy
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class Sampler(Node):
    def __init__(self, save_path, frames):
        super().__init__("cam_brightness")
        self.n = 0
        self.save_path = save_path
        self.frames = frames
        self.create_subscription(
            CompressedImage, "/camera/image_raw/compressed", self.on_frame, 10
        )

    def on_frame(self, msg):
        im = Image.open(io.BytesIO(msg.data)).convert("RGB")
        small = im.convert("L").resize((48, 48))
        luma = list(small.getdata())
        rgb = list(im.resize((32, 32)).getdata())
        n = len(rgb)
        print(
            "frame %2d  size=%s  mean_luma=%5.1f  max=%3d  meanRGB=(%.1f,%.1f,%.1f)"
            % (
                self.n,
                im.size,
                sum(luma) / len(luma),
                max(luma),
                sum(p[0] for p in rgb) / n,
                sum(p[1] for p in rgb) / n,
                sum(p[2] for p in rgb) / n,
            )
        )
        if self.save_path:
            im.save(self.save_path)
        self.n += 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--save", default="", help="この名前に最後の 1 枚を保存する")
    a = ap.parse_args()

    rclpy.init()
    s = Sampler(a.save, a.frames)
    while rclpy.ok() and s.n < a.frames:
        rclpy.spin_once(s, timeout_sec=0.5)
    if s.n < a.frames:
        print("映像が届かない（camera_node が止まっている）", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
