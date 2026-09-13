"""AI-CAR Webダッシュボードノード。

FastAPI サーバーを別スレッドで起動し、ブラウザからの手動操作コマンドを
/cmd_vel へ publish し、センサートピックのテレメトリを WebSocket で配信する。
"""

import asyncio
import json
import math
import os
import threading

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


MAX_SCAN_POINTS = 720


class CmdVelRequest(BaseModel):
    """ブラウザから受け取る速度指令。"""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


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
        self.declare_parameter('scan_angle_offset_deg', 180.0)

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        self.camera_stream_rate = float(self.get_parameter('camera_stream_rate').value)
        self.obstacle_guard = bool(self.get_parameter('obstacle_guard').value)
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
        self._frame = None
        self._frame_stamp = 0.0
        self._frame_count = 0

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
        with self._lock:
            self._frame = bytes(msg.data)
            self._frame_stamp = self.get_clock().now().nanoseconds * 1e-9
            self._frame_count += 1

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
        }

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

        async def frames():
            last = -1
            while True:
                frame, count = node.latest_frame()
                if frame is not None and count != last:
                    last = count
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(frame)).encode()
                           + b'\r\n\r\n' + frame + b'\r\n')
                await asyncio.sleep(interval)

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
