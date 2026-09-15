"""AI-CAR Webダッシュボードノード。

FastAPI サーバーを別スレッドで起動し、ブラウザからの手動操作コマンドを
/cmd_vel へ publish し、センサートピックのテレメトリを WebSocket で配信する。
"""

import asyncio
import json
import math
import os
import threading
import time
from collections import deque

import rclpy
import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import String

from ai_car_web.gpio_pinout import build_pinout


MAX_SCAN_POINTS = 720


class CmdVelRequest(BaseModel):
    """ブラウザから受け取る速度指令。"""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


def _jpeg_dimensions(data: bytes):
    """JPEG のフレームヘッダ（SOFn）から (幅, 高さ) を読む。取れなければ None。"""
    i = 2
    end = len(data)
    while i + 9 < end:
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            height = int.from_bytes(data[i + 5:i + 7], 'big')
            width = int.from_bytes(data[i + 7:i + 9], 'big')
            return width, height
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        i += 2 + int.from_bytes(data[i + 2:i + 4], 'big')
    return None


def quaternion_to_euler(x, y, z, w):
    """クォータニオンを roll/pitch/yaw [rad] に変換する。"""
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class DashboardNode(Node):
    """Webダッシュボード用の ROS2 ノード。"""

    def __init__(self):
        super().__init__('dashboard_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('system_status_topic', '/system_status')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_stream_rate', 10.0)
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('obstacle_guard', True)
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('cmd_timeout', 0.7)
        self.declare_parameter('telemetry_rate', 5.0)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('gpio_config', '')
        # ゲームパッド（joy_teleop_node）からの正規化指令。空で無効
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('joy_timeout', 1.0)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        self.camera_stream_rate = float(self.get_parameter('camera_stream_rate').value)
        self.obstacle_guard = bool(self.get_parameter('obstacle_guard').value)
        self.gpio_config = self.get_parameter('gpio_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'gpio_pins.yaml')
        # LiDAR の 0° とロボット前方のずれ（取り付け向きの補正）
        self.scan_angle_offset = math.radians(
            float(self.get_parameter('scan_angle_offset_deg').value))

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.status_pub = self.create_publisher(String, '~/status', 10)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._imu_cb, sensor_qos)
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            String, self.get_parameter('system_status_topic').value,
            self._system_cb, 10)
        self.create_subscription(
            CompressedImage, self.get_parameter('camera_topic').value,
            self._camera_cb, sensor_qos)
        self.create_subscription(
            String, self.get_parameter('obstacle_topic').value, self._obstacle_cb, 10)
        self.joy_timeout = float(self.get_parameter('joy_timeout').value)
        joy_topic = self.get_parameter('joy_cmd_topic').value
        if joy_topic:
            self.create_subscription(Twist, joy_topic, self._joy_cb, 10)

        self._lock = threading.Lock()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._cmd_active = False
        self._scan = None
        self._imu = None
        self._odom = None
        self._system = None
        self._obstacle = None
        self._obstacle_stamp = 0.0
        self._joy_stamp = 0.0
        self._joy_moving = False
        self._frame = None
        self._frame_stamp = 0.0
        self._frame_count = 0
        self._frame_bytes = 0
        self._frame_size = None
        self._frame_times = deque(maxlen=60)

        self.create_timer(0.1, self._watchdog_cb)
        self.get_logger().info(
            f'Webダッシュボードを起動します: http://{self.host}:{self.port}')

    # --- サブスクライバ ---
    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        step = max(1, len(msg.ranges) // MAX_SCAN_POINTS)
        points = []
        for i in range(0, len(msg.ranges), step):
            r = msg.ranges[i]
            if not math.isfinite(r) or r <= msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment + self.scan_angle_offset
            points.append([round(r * math.cos(angle), 3),
                           round(r * math.sin(angle), 3)])
        with self._lock:
            self._scan = {
                'stamp': self._stamp_to_sec(msg.header.stamp),
                'count': len(msg.ranges),
                'range_min': round(min(ranges), 3) if ranges else None,
                'range_max': round(max(ranges), 3) if ranges else None,
                'front': self._front_distance(msg, self.scan_angle_offset),
                'points': points,
            }

    def _imu_cb(self, msg: Imu):
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self._lock:
            self._imu = {
                'stamp': self._stamp_to_sec(msg.header.stamp),
                'roll_deg': round(math.degrees(roll), 2),
                'pitch_deg': round(math.degrees(pitch), 2),
                'yaw_deg': round(math.degrees(yaw), 2),
                'angular_velocity_z': round(msg.angular_velocity.z, 4),
                'linear_acceleration_x': round(msg.linear_acceleration.x, 4),
            }

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self._lock:
            self._odom = {
                'stamp': self._stamp_to_sec(msg.header.stamp),
                'x': round(p.x, 3),
                'y': round(p.y, 3),
                'yaw_deg': round(math.degrees(yaw), 2),
                'linear_x': round(msg.twist.twist.linear.x, 3),
                'angular_z': round(msg.twist.twist.angular.z, 3),
            }

    def _obstacle_cb(self, msg: String):
        try:
            obstacle = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('obstacle_status の JSON を解析できません')
            return
        with self._lock:
            self._obstacle = obstacle
            self._obstacle_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _forward_scale(self) -> float:
        """障害物判定に応じた前進方向の速度制限係数を返す。"""
        if not self.obstacle_guard:
            return 1.0
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            obstacle = self._obstacle
            age = now - self._obstacle_stamp if self._obstacle_stamp else None
        if not obstacle or age is None or age > 2.0:
            return 1.0
        return float(obstacle.get('speed_scale', 1.0))

    def _system_cb(self, msg: String):
        try:
            system = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('system_status の JSON を解析できません')
            return
        with self._lock:
            self._system = system

    def _camera_cb(self, msg: CompressedImage):
        data = bytes(msg.data)
        with self._lock:
            self._frame = data
            self._frame_stamp = self.get_clock().now().nanoseconds * 1e-9
            self._frame_count += 1
            self._frame_bytes = len(data)
            self._frame_times.append(time.monotonic())
            # 解像度は起動直後と 5 秒ごとだけ読む（毎フレーム解析する必要はない）
            if self._frame_size is None or self._frame_count % 150 == 0:
                self._frame_size = _jpeg_dimensions(data)

    def latest_frame(self):
        """最新の JPEG フレームと逗番を返す。未受信なら (None, 0)。"""
        with self._lock:
            return self._frame, self._frame_count

    def _camera_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        age = now - self._frame_stamp if self._frame is not None else None
        return {
            'available': self._frame is not None and age is not None and age < 3.0,
            'topic': self.get_parameter('camera_topic').value,
            'frames': self._frame_count,
            'age_s': round(age, 2) if age is not None else None,
            'width': self._frame_size[0] if self._frame_size else None,
            'height': self._frame_size[1] if self._frame_size else None,
            'kb': round(self._frame_bytes / 1024.0, 1) if self._frame_bytes else None,
            'fps': self._frame_fps(),
            'stream_fps': self.camera_stream_rate,
        }

    def _frame_fps(self):
        """直近の受信間隔から実測フレームレートを返す。"""
        now = time.monotonic()
        recent = [t for t in self._frame_times if now - t <= 3.0]
        if len(recent) < 2:
            return None
        span = recent[-1] - recent[0]
        return round((len(recent) - 1) / span, 1) if span > 0 else None

    @staticmethod
    def _stamp_to_sec(stamp):
        return round(stamp.sec + stamp.nanosec * 1e-9, 3)

    @staticmethod
    def _front_distance(msg: LaserScan, offset: float = 0.0):
        """正面 (±5deg) の最短距離 [m] を返す。"""
        if not msg.ranges or msg.angle_increment == 0.0:
            return None
        half_width = math.radians(5.0)
        best = None
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min:
                continue
            angle = msg.angle_min + i * msg.angle_increment + offset
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= half_width and (best is None or r < best):
                best = r
        return round(best, 3) if best is not None else None

    def _joy_cb(self, msg: Twist):
        """ゲームパッドの正規化指令を Web 操作と同じ経路で /cmd_vel に変換する。"""
        moving = (abs(msg.linear.x) > 0.0 or abs(msg.linear.y) > 0.0
                  or abs(msg.angular.z) > 0.0)
        with self._lock:
            self._joy_stamp = self.get_clock().now().nanoseconds * 1e-9
            was_moving = self._joy_moving
            self._joy_moving = moving
        # 停止中はニュートラルへ戻った瞬間だけ停止を送り、Web 操作の指令を上書きしない
        if moving or was_moving:
            self.publish_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    # --- 速度指令 ---
    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        """正規化済み (-1.0〜1.0) の指令値を最大速度にスケールして publish する。"""
        twist = Twist()
        forward = self._clamp(linear_x)
        if forward > 0.0:
            forward *= self._forward_scale()
        twist.linear.x = forward * self.max_linear
        twist.linear.y = self._clamp(linear_y) * self.max_linear
        twist.angular.z = self._clamp(angular_z) * self.max_angular
        self.cmd_vel_pub.publish(twist)
        with self._lock:
            self._last_cmd = twist
            self._last_cmd_time = self.get_clock().now().nanoseconds * 1e-9
            self._cmd_active = True
        return {
            'linear_x': twist.linear.x,
            'linear_y': twist.linear.y,
            'angular_z': twist.angular.z,
        }

    def stop(self):
        """停止指令を publish する。"""
        return self.publish_cmd_vel(0.0, 0.0, 0.0)

    @staticmethod
    def _clamp(value: float) -> float:
        return max(-1.0, min(1.0, float(value)))

    def _watchdog_cb(self):
        """一定時間指令が来ない場合に停止指令を送る安全機構。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            active = self._cmd_active
            elapsed = now - self._last_cmd_time
            moving = (abs(self._last_cmd.linear.x) > 0.0
                      or abs(self._last_cmd.linear.y) > 0.0
                      or abs(self._last_cmd.angular.z) > 0.0)
        if active and moving and elapsed > self.cmd_timeout:
            self.cmd_vel_pub.publish(Twist())
            with self._lock:
                self._last_cmd = Twist()
                self._cmd_active = False
            self.get_logger().warn('指令タイムアウトのため停止しました')

    def telemetry(self):
        """WebSocket / REST で返すテレメトリを組み立てる。"""
        with self._lock:
            return {
                'stamp': round(self.get_clock().now().nanoseconds * 1e-9, 3),
                'cmd_vel': {
                    'linear_x': round(self._last_cmd.linear.x, 3),
                    'linear_y': round(self._last_cmd.linear.y, 3),
                    'angular_z': round(self._last_cmd.angular.z, 3),
                },
                'scan': self._scan,
                'imu': self._imu,
                'odom': self._odom,
                'system': self._system,
                'obstacle': self._obstacle,
                'camera': self._camera_state(),
                'joy': {
                    'connected': (self.get_clock().now().nanoseconds * 1e-9
                                  - self._joy_stamp) < self.joy_timeout,
                    'active': self._joy_moving,
                },
                'limits': {
                    'max_linear_speed': self.max_linear,
                    'max_angular_speed': self.max_angular,
                },
            }


def create_app(node: DashboardNode) -> FastAPI:
    """ダッシュボードの FastAPI アプリを生成する。"""
    static_dir = os.path.join(
        get_package_share_directory('ai_car_web'), 'static')
    app = FastAPI(title='AI-CAR Dashboard')

    @app.get('/')
    def index():
        return FileResponse(os.path.join(static_dir, 'index.html'))

    @app.get('/api/status')
    def status():
        return node.telemetry()

    @app.get('/api/gpio')
    def gpio():
        return build_pinout(node.gpio_config)

    @app.post('/api/cmd_vel')
    def cmd_vel(req: CmdVelRequest):
        return node.publish_cmd_vel(req.linear_x, req.linear_y, req.angular_z)

    @app.post('/api/stop')
    def stop():
        return node.stop()

    @app.get('/api/camera/snapshot')
    def snapshot():
        frame, _ = node.latest_frame()
        if frame is None:
            return Response(status_code=503)
        return Response(content=frame, media_type='image/jpeg')

    @app.get('/api/camera/stream')
    def stream():
        interval = 1.0 / max(node.camera_stream_rate, 0.1)

        # 配信周期と同じ間隔で寝ると位相ずれで新フレームを取り逃がし実効レートが
        # 半減するため、短い周期で監視して配信間隔だけを守る。
        poll = min(interval / 4.0, 0.005)

        async def frames():
            last = -1
            next_at = 0.0
            while True:
                frame, count = node.latest_frame()
                now = time.monotonic()
                if frame is not None and count != last and now >= next_at:
                    last = count
                    next_at = now + interval
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(frame)).encode()
                           + b'\r\n\r\n' + frame + b'\r\n')
                await asyncio.sleep(poll)

        return StreamingResponse(
            frames(),
            media_type='multipart/x-mixed-replace; boundary=frame')

    @app.websocket('/ws')
    async def telemetry_ws(websocket: WebSocket):
        await websocket.accept()
        interval = 1.0 / max(node.telemetry_rate, 0.1)
        try:
            while True:
                await websocket.send_text(json.dumps(node.telemetry()))
                await asyncio.sleep(interval)
        except (WebSocketDisconnect, ConnectionError):
            pass

    app.mount('/static', StaticFiles(directory=static_dir), name='static')
    return app


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    app = create_app(node)

    config = uvicorn.Config(
        app, host=node.host, port=node.port, log_level='info', access_log=False)
    server = uvicorn.Server(config)
    server_thread = threading.Thread(target=server.run, daemon=True)
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        server_thread.join(timeout=5.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
