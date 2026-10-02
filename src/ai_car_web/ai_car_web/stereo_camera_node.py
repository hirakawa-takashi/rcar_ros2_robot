#!/usr/bin/env python3
"""USB の 2 眼カメラ（ELP 3D USB Camera）の片目の映像を JPEG で publish するノード。

カメラは左右の画像を横に並べた 1 枚（例: 2560x720 = 1280x720 x 2）を MJPG で送ってくる。
上下逆さまに付けたときは rotate_180 を true にする。1 枚全体を 180° 回すと左右の画像も
入れ替わるので、回したあとの左半分が左目、右半分が右目になる。
eye で選んだ片目を output_width x output_height に縮めて、camera_ros と同じ
/camera/image_raw/compressed（JPEG）へ出す。ダッシュボード・perception_node・
floor_obstacle_node はそのまま使える。
"""

import threading
import time

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


def select_eye(frame, eye, rotate_180):
    """横並びの 2 眼画像から片目を取り出す（rotate_180 なら 180° 回したあとの左右）。"""
    half = frame.shape[1] // 2
    left_raw, right_raw = frame[:, :half], frame[:, half:]
    if rotate_180:
        side = right_raw if eye == 'left' else left_raw
        return cv2.rotate(side, cv2.ROTATE_180)
    return left_raw if eye == 'left' else right_raw


class StereoCameraNode(Node):
    """2 眼カメラを V4L2 で読み、片目の JPEG を publish する。"""

    def __init__(self):
        super().__init__('stereo_camera_node')
        self.declare_parameter(
            'device', '/dev/v4l/by-id/usb-3D_USB_Camera_3D_USB_Camera_01.00.00-video-index0')
        self.declare_parameter('width', 2560)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('rotate_180', True)
        self.declare_parameter('eye', 'left')
        self.declare_parameter('output_width', 960)
        self.declare_parameter('output_height', 540)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera_link')
        g = self.get_parameter
        self.device = g('device').value
        self.width = int(g('width').value)
        self.height = int(g('height').value)
        self.fps = float(g('fps').value)
        self.rotate_180 = bool(g('rotate_180').value)
        self.eye = 'right' if g('eye').value == 'right' else 'left'
        self.out_size = (int(g('output_width').value), int(g('output_height').value))
        self.jpeg_params = [cv2.IMWRITE_JPEG_QUALITY, int(g('jpeg_quality').value)]
        self.frame_id = g('frame_id').value
        self.pub = self.create_publisher(CompressedImage, g('image_topic').value, 10)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open(self):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.get_logger().info(
            f'2 眼カメラ接続: {self.device} {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} → {self.eye} 目 '
            f'{self.out_size[0]}x{self.out_size[1]}（180° 回転: {self.rotate_180}）')
        return cap

    def _loop(self):
        cap = None
        last_warn = 0.0
        while self._running and rclpy.ok():
            if cap is None:
                cap = self._open()
                if cap is None:
                    if time.monotonic() - last_warn >= 10.0:
                        self.get_logger().warn(f'2 眼カメラを開けません: {self.device}')
                        last_warn = time.monotonic()
                    time.sleep(2.0)
                    continue
            ok, frame = cap.read()
            if not ok or frame is None:
                self.get_logger().warn('2 眼カメラの読み取りに失敗。開き直します')
                cap.release()
                cap = None
                time.sleep(1.0)
                continue
            image = cv2.resize(select_eye(frame, self.eye, self.rotate_180), self.out_size,
                               interpolation=cv2.INTER_AREA)
            ok, jpeg = cv2.imencode('.jpg', image, self.jpeg_params)
            if not ok:
                continue
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.format = 'jpeg'
            msg.data = jpeg.tobytes()
            if not rclpy.ok():
                break
            self.pub.publish(msg)
        if cap is not None:
            cap.release()

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
