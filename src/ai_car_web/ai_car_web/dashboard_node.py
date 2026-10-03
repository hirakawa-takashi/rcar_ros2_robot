"""AI-CAR Webダッシュボードノード。

FastAPI サーバーを別スレッドで起動し、ブラウザ・ゲームパッドからの手動操作コマンドを
手動指令トピック（既定 /cmd_vel_manual。drive_mode_node がモードに応じて /cmd_vel へ中継）
へ publish し、センサートピックのテレメトリを WebSocket で配信する。
"""

import asyncio
import hmac
import json
import math
import os
import struct
import subprocess
import threading
import time
import zlib
from collections import deque

import rclpy
import uvicorn
import yaml
from ament_index_python.packages import get_package_share_directory
from fastapi import Depends, FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import String

from ai_car_web.architecture import load_architecture
from ai_car_web.dev_diary import find_repo, load_dev_diary
from ai_car_web.gpio_pinout import build_pinout
from ai_car_web.motor_hat import load_motor_hat

try:
    from slam_toolbox.srv import SaveMap
except ImportError:
    SaveMap = None


MAX_SCAN_POINTS = 720


class CmdVelRequest(BaseModel):
    """ブラウザから受け取る速度指令。"""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class DriveModeRequest(BaseModel):
    """ブラウザから受け取る運転モード切替要求（manual / auto / stop）。"""

    mode: str


class JetsonDetection(BaseModel):
    """Jetson の物体検出 1 件。box は画像に対する正規化座標 [x_min, y_min, x_max, y_max]。"""

    label: str
    prompt: str = ''
    score: float
    box: list[float]


class JetsonDetectionsRequest(BaseModel):
    """Jetson（NanoOWL など）から受け取るカメラ画像 1 枚分の検出結果。"""

    detections: list[JetsonDetection] = []
    model: str = ''
    inference_ms: float | None = None


class JetsonStatusRequest(BaseModel):
    """Jetson（jetson/status_reporter）から受け取る温度・使用率・メモリ・電力・AI の部品・更新の状況。"""

    hostname: str = ''
    os: str = ''
    l4t: str = ''
    power_mode: str = ''
    uptime_s: float | None = None
    cpu_percent: float | None = None
    cpu_freq_mhz: float | None = None
    gpu_percent: float | None = None
    gpu_freq_mhz: float | None = None
    temperatures_c: dict[str, float] = {}
    mem_total_mb: float | None = None
    mem_used_mb: float | None = None
    mem_available_mb: float | None = None
    swap_total_mb: float | None = None
    swap_used_mb: float | None = None
    input_volt: float | None = None
    power_w: float | None = None
    fan_percent: float | None = None
    fan_rpm: float | None = None
    disk_total_gb: float | None = None
    disk_used_gb: float | None = None
    services: dict[str, bool] = {}
    llm_models: list[str] = []
    llm_size_mb: float | None = None
    llm_vram_mb: float | None = None
    updates_pending: int | None = None
    updates_held: int | None = None
    security_pending: int | None = None
    reboot_required: bool | None = None
    last_upgrade: float | None = None
    ips: list[dict[str, str]] = []
    can_reboot: bool | None = None
    can_upgrade: bool | None = None
    upgrade_running: bool | None = None
    upgrade_result: str = ''
    upgrade_percent: int | None = None
    upgrade_phase: str = ''


JETSON_STATUS_TIMEOUT = 5.0
JETSON_REBOOT_WINDOW = 10.0
PI_REBOOT_DELAY = 10.0
REBOOT_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'reboot']
UPGRADE_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'start', '--no-block', 'ai-car-upgrade.service']


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


def _occupancy_table():
    """OccupancyGrid の値（int8 を符号なしで見たもの）→ 灰色の濃さ。不明 205、空き 254、占有 0。"""
    table = bytearray(256)
    for u in range(256):
        table[u] = 205 if u > 100 else 254 - round(u * 254 / 100)
    return bytes(table)


OCCUPANCY_TABLE = _occupancy_table()


def occupancy_png(width, height, data):
    """OccupancyGrid の data を 8 bit グレーの PNG にする（画像の上が地図の +y）。"""
    pixels = bytes(data).translate(OCCUPANCY_TABLE)
    rows = b''.join(b'\x00' + pixels[r * width:(r + 1) * width] for r in range(height - 1, -1, -1))

    def chunk(kind, body):
        return (struct.pack('>I', len(body)) + kind + body
                + struct.pack('>I', zlib.crc32(kind + body) & 0xFFFFFFFF))

    header = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header)
            + chunk(b'IDAT', zlib.compress(rows, 6)) + chunk(b'IEND', b''))


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
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_manual')
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
        # LiDAR 点群をブラウザに送るときの上限点数（間引き）
        self.declare_parameter('scan_max_points', MAX_SCAN_POINTS)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('gpio_config', '')
        self.declare_parameter('motor_hat_config', '')
        self.declare_parameter('architecture_config', '')
        self.declare_parameter('params_config', '')
        self.declare_parameter('repo_dir', '')
        # ゲームパッド（joy_teleop_node）からの正規化指令。空で無効
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('joy_timeout', 1.0)
        # 運転モード（drive_mode_node）。空で無効
        self.declare_parameter('drive_mode_topic', '/drive_mode')
        self.declare_parameter('drive_mode_request_topic', '/drive_mode_request')
        self.declare_parameter('api_token', '')
        # 自律走行の状態（autonomy_node）。空で無効
        self.declare_parameter('autonomy_status_topic', '/autonomy_status')
        # 落下防止センサー（cliff_node）。空で無効
        self.declare_parameter('cliff_topic', '/cliff_status')
        # SLAM（slam_toolbox）の地図と自己位置。空で無効
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('slam_pose_topic', '/pose')
        self.declare_parameter('slam_save_service', '/slam_toolbox/save_map')
        # 地図の保存先（空なら ~/maps）
        self.declare_parameter('map_save_dir', '')
        # Jetson から POST /api/jetson/detections で受けた検出を流すトピック。空で無効
        self.declare_parameter('jetson_detections_topic', '/jetson_detections')

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        self.scan_max_points = max(1, int(self.get_parameter('scan_max_points').value))
        self.camera_stream_rate = float(self.get_parameter('camera_stream_rate').value)
        self.obstacle_guard = bool(self.get_parameter('obstacle_guard').value)
        self.api_token = str(self.get_parameter('api_token').value) or os.environ.get(
            'AI_CAR_API_TOKEN', '')
        self.gpio_config = self.get_parameter('gpio_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'gpio_pins.yaml')
        self.motor_hat_config = self.get_parameter('motor_hat_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'motor_hat.yaml')
        self.architecture_config = self.get_parameter('architecture_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'architecture.yaml')
        self.params_config = self.get_parameter('params_config').value or os.path.join(
            get_package_share_directory('ai_car_web'), 'config', 'dashboard.yaml')
        self.repo_dir = find_repo(self.get_parameter('repo_dir').value
                                  or get_package_share_directory('ai_car_web'))
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
        jetson_topic = self.get_parameter('jetson_detections_topic').value
        self.jetson_pub = (self.create_publisher(String, jetson_topic, 10)
                           if jetson_topic else None)

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
        mode_topic = self.get_parameter('drive_mode_topic').value
        self.mode_request_pub = None
        if mode_topic:
            self.create_subscription(
                String, mode_topic, self._drive_mode_cb,
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
            self.mode_request_pub = self.create_publisher(
                String, self.get_parameter('drive_mode_request_topic').value, 10)
        autonomy_topic = self.get_parameter('autonomy_status_topic').value
        if autonomy_topic:
            self.create_subscription(String, autonomy_topic, self._autonomy_cb, 10)
        cliff_topic = self.get_parameter('cliff_topic').value
        if cliff_topic:
            self.create_subscription(String, cliff_topic, self._cliff_cb, 10)
        map_topic = self.get_parameter('map_topic').value
        if map_topic:
            self.create_subscription(
                OccupancyGrid, map_topic, self._map_cb,
                QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        slam_pose_topic = self.get_parameter('slam_pose_topic').value
        if slam_pose_topic:
            self.create_subscription(
                PoseWithCovarianceStamped, slam_pose_topic, self._slam_pose_cb, 10)
        self.map_save_dir = self.get_parameter('map_save_dir').value or os.path.join(
            os.path.expanduser('~'), 'maps')
        self.slam_save_client = None
        if SaveMap is not None:
            self.slam_save_client = self.create_client(
                SaveMap, self.get_parameter('slam_save_service').value)

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
        self._joy_cmd = (0.0, 0.0, 0.0)
        self._drive_mode = None
        self._drive_mode_stamp = 0.0
        self._autonomy = None
        self._autonomy_stamp = 0.0
        self._cliff = None
        self._cliff_stamp = 0.0
        self._jetson_host = None
        self._jetson_host_stamp = 0.0
        self._jetson_reboot_at = None
        self._jetson_upgrade_at = None
        self._map = None
        self._map_png = None
        self._map_stamp = 0.0
        self._slam_pose = None
        self._slam_pose_stamp = 0.0
        self._map_saved = None
        self._frame = None
        self._frame_stamp = 0.0
        self._frame_count = 0
        self._frame_bytes = 0
        self._frame_size = None
        self._frame_times = deque(maxlen=60)

        self.create_timer(0.1, self._watchdog_cb)
        if not self.api_token:
            self.get_logger().warn(
                'API 認証が無効です（api_token 未設定）: 同一ネットワークの誰でも操作できます')
        self.get_logger().info(
            f'Webダッシュボードを起動します: http://{self.host}:{self.port}')

    # --- サブスクライバ ---
    def _scan_cb(self, msg: LaserScan):
        ranges = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        step = max(1, len(msg.ranges) // self.scan_max_points)
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

    def _map_cb(self, msg: OccupancyGrid):
        info = msg.info
        png = occupancy_png(info.width, info.height, msg.data)
        _, _, yaw = quaternion_to_euler(
            info.origin.orientation.x, info.origin.orientation.y,
            info.origin.orientation.z, info.origin.orientation.w)
        with self._lock:
            rev = (self._map or {}).get('rev', 0) + 1
            self._map = {
                'rev': rev,
                'width': info.width,
                'height': info.height,
                'resolution': round(info.resolution, 4),
                'origin_x': round(info.origin.position.x, 3),
                'origin_y': round(info.origin.position.y, 3),
                'origin_yaw_deg': round(math.degrees(yaw), 2),
                'frame_id': msg.header.frame_id,
            }
            self._map_png = png
            self._map_stamp = time.time()

    def _slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        with self._lock:
            self._slam_pose = {
                'x': round(p.x, 3),
                'y': round(p.y, 3),
                'yaw_deg': round(math.degrees(yaw), 2),
            }
            self._slam_pose_stamp = time.time()

    def _slam_state(self):
        now = time.time()
        map_age = now - self._map_stamp if self._map_stamp else None
        pose_age = now - self._slam_pose_stamp if self._slam_pose_stamp else None
        return {
            'alive': pose_age is not None and pose_age < 5.0,
            'map': self._map,
            'map_age': round(map_age, 1) if map_age is not None else None,
            'pose': self._slam_pose,
            'pose_age': round(pose_age, 1) if pose_age is not None else None,
            'can_save': self.slam_save_client is not None,
            'saved': self._map_saved,
        }

    def map_png(self):
        with self._lock:
            return self._map_png

    def save_map(self, timeout: float = 10.0):
        """slam_toolbox の save_map を呼んで、地図を map_save_dir に .pgm / .yaml で保存する。"""
        if self.slam_save_client is None:
            return {'ok': False, 'reason': 'slam_toolbox が入っていない'}
        if not self.slam_save_client.service_is_ready():
            return {'ok': False, 'reason': 'slam_toolbox が起動していない（use_slam:=true）'}
        os.makedirs(self.map_save_dir, exist_ok=True)
        name = os.path.join(self.map_save_dir, time.strftime('map_%Y%m%d_%H%M%S'))
        request = SaveMap.Request()
        request.name = String(data=name)
        done = threading.Event()
        future = self.slam_save_client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return {'ok': False, 'reason': '保存の応答がない'}
        result = future.result()
        if result is None:
            ok, reason = False, '保存の応答が空'
        else:
            ok = result.result == SaveMap.Response.RESULT_SUCCESS
            reason = '' if ok else f'保存に失敗（result {result.result}）'
        saved = {'ok': ok, 'name': name, 'stamp': round(time.time(), 1), 'reason': reason}
        with self._lock:
            self._map_saved = saved
        return saved

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

    def _drive_mode_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._drive_mode = payload
            self._drive_mode_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _autonomy_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._autonomy = payload
            self._autonomy_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _cliff_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._cliff = payload
            self._cliff_stamp = self.get_clock().now().nanoseconds * 1e-9

    def request_drive_mode(self, mode: str):
        """運転モードの切替を drive_mode_node へ要求する。"""
        mode = mode.strip().lower()
        if mode not in ('manual', 'auto', 'stop'):
            return {'ok': False, 'error': f'不明なモード: {mode}'}
        if self.mode_request_pub is None:
            return {'ok': False, 'error': '運転モード管理が無効です'}
        if mode != 'manual':
            # 自動 / 停止へ切り替える前に Web 操作の指令を止める
            self.neutralize()
        self._publish_mode_request(mode)
        self.get_logger().info(f'運転モード切替要求: {mode}')
        return {'ok': True, 'mode': mode}

    def _publish_mode_request(self, mode: str):
        msg = String()
        msg.data = json.dumps({'mode': mode, 'source': 'dashboard'})
        self.mode_request_pub.publish(msg)

    def _drive_mode_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._drive_mode is None:
            return None
        age = now - self._drive_mode_stamp
        state = dict(self._drive_mode)
        state['alive'] = age < 3.0
        state['age_s'] = round(age, 1)
        return state

    def _autonomy_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._autonomy is None:
            return None
        age = now - self._autonomy_stamp
        state = dict(self._autonomy)
        state['alive'] = age < 3.0
        state['age_s'] = round(age, 1)
        return state

    def _cliff_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._cliff is None:
            return None
        age = now - self._cliff_stamp
        state = dict(self._cliff)
        state['alive'] = age < 3.0
        state['age_s'] = round(age, 1)
        return state

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
            self._joy_cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        # 停止中はニュートラルへ戻った瞬間だけ停止を送り、Web 操作の指令を上書きしない
        if moving or was_moving:
            self.publish_cmd_vel(msg.linear.x, msg.linear.y, msg.angular.z)

    # --- 速度指令 ---
    def publish_jetson_detections(self, req: JetsonDetectionsRequest):
        """Jetson の検出結果を整えて jetson_detections_topic へ publish する。"""
        if self.jetson_pub is None:
            raise HTTPException(status_code=503, detail='jetson_detections_topic が無効です')
        detections = []
        for d in req.detections[:20]:
            if len(d.box) != 4:
                continue
            x_min, y_min, x_max, y_max = (max(0.0, min(1.0, float(v))) for v in d.box)
            if x_max <= x_min or y_max <= y_min:
                continue
            detections.append({
                'label': d.label[:32],
                'prompt': d.prompt[:64],
                'score': round(float(d.score), 3),
                'box': [round(x_min, 3), round(y_min, 3), round(x_max, 3), round(y_max, 3)],
            })
        msg = String()
        msg.data = json.dumps({
            'stamp': round(time.time(), 3),
            'model': req.model[:64],
            'inference_ms': req.inference_ms,
            'detections': detections,
        })
        self.jetson_pub.publish(msg)
        return {'ok': True, 'count': len(detections)}

    def update_jetson_status(self, req: JetsonStatusRequest):
        """Jetson の状態を、文字の長さと項目の数を絞ってから覚える。"""
        status = req.model_dump()
        for key in ('hostname', 'os', 'l4t', 'power_mode', 'upgrade_result', 'upgrade_phase'):
            status[key] = status[key][:64]
        status['temperatures_c'] = {k[:16]: round(v, 1)
                                    for k, v in list(req.temperatures_c.items())[:8]}
        status['services'] = {k[:16]: v for k, v in list(req.services.items())[:8]}
        status['llm_models'] = [m[:64] for m in req.llm_models[:4]]
        status['ips'] = [{'iface': str(ip.get('iface', ''))[:16], 'addr': str(ip.get('addr', ''))[:64]}
                         for ip in req.ips[:8]]
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._jetson_host = status
            self._jetson_host_stamp = now
            reboot = (self._jetson_reboot_at is not None
                      and now - self._jetson_reboot_at < JETSON_REBOOT_WINDOW)
            self._jetson_reboot_at = None
            upgrade = (self._jetson_upgrade_at is not None
                       and now - self._jetson_upgrade_at < JETSON_REBOOT_WINDOW)
            self._jetson_upgrade_at = None
        if reboot:
            self.get_logger().warn('Jetson に再起動を伝えました')
        if upgrade:
            self.get_logger().info('Jetson に更新を伝えました')
        return {'ok': True, 'reboot': reboot, 'upgrade': upgrade}

    def request_jetson_upgrade(self):
        """次に Jetson から状態が届いたときの返事で、更新（NVIDIA の部品以外）を頼む。"""
        with self._lock:
            state = self._jetson_host_state()
            if not state or not state['alive']:
                raise HTTPException(status_code=409, detail='Jetson が未接続です')
            if not state.get('can_upgrade'):
                raise HTTPException(
                    status_code=503,
                    detail='Jetson に jetson-upgrade.sudoers が入っていません')
            if state.get('upgrade_running'):
                raise HTTPException(status_code=409, detail='Jetson は更新中です')
            self._jetson_upgrade_at = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info('ダッシュボードから Jetson の更新を受け付けました')
        return {'ok': True}

    def request_jetson_reboot(self):
        """次に Jetson から状態が届いたときの返事で、再起動を頼む。"""
        with self._lock:
            state = self._jetson_host_state()
            if not state or not state['alive']:
                raise HTTPException(status_code=409, detail='Jetson が未接続です')
            if not state.get('can_reboot'):
                raise HTTPException(
                    status_code=503,
                    detail='Jetson に jetson-reboot.sudoers が入っていません')
            self._jetson_reboot_at = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn('ダッシュボードから Jetson の再起動を受け付けました')
        return {'ok': True}

    def _moving(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            web = self._cmd_active and (abs(self._last_cmd.linear.x) > 0.0
                                        or abs(self._last_cmd.linear.y) > 0.0
                                        or abs(self._last_cmd.angular.z) > 0.0)
            mode = self._drive_mode if now - self._drive_mode_stamp < 3.0 else None
            return web or self._joy_moving or bool(mode and mode.get('mode') == 'auto')

    def reboot_pi(self):
        """止まっているときだけ、ラズパイを再起動する（10 秒後。返事を先に返す）。"""
        if self._moving():
            raise HTTPException(status_code=409, detail='走行中は再起動できません（先に止めてください）')
        try:
            allowed = subprocess.run(['sudo', '-n', '-l', *REBOOT_CMD[2:]], capture_output=True,
                                     timeout=5, check=False).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            allowed = False
        if not allowed:
            raise HTTPException(status_code=503,
                                detail='ラズパイに ai-car-reboot.sudoers が入っていません')
        self.stop()
        self.get_logger().warn(f'ダッシュボードからラズパイの再起動を受け付けました（{PI_REBOOT_DELAY:.0f} 秒後）')
        threading.Thread(target=self._reboot_pi_later, daemon=True).start()
        return {'ok': True}

    def upgrade_pi(self):
        """止まっているときだけ、ラズパイの更新（ai-car-upgrade.service）を始める。"""
        if self._moving():
            raise HTTPException(status_code=409, detail='走行中は更新できません（先に止めてください）')
        with self._lock:
            updates = (self._system or {}).get('updates') or {}
        if not updates.get('can_upgrade'):
            raise HTTPException(status_code=503,
                                detail='ラズパイに ai-car-upgrade.sudoers が入っていません')
        if updates.get('upgrade_running'):
            raise HTTPException(status_code=409, detail='ラズパイは更新中です')
        try:
            ok = subprocess.run(UPGRADE_CMD, capture_output=True, timeout=30,
                                check=False).returncode == 0
        except (OSError, subprocess.TimeoutExpired):
            ok = False
        if not ok:
            raise HTTPException(status_code=500, detail='更新を始められませんでした')
        self.get_logger().info('ダッシュボードからラズパイの更新を始めました')
        return {'ok': True}

    def _reboot_pi_later(self):
        """10 秒待ち（そのあいだも Jetson の再起動を受け付ける）、Jetson への再起動の頼みが残っていれば伝え終わるまで待ってから再起動する。"""
        time.sleep(PI_REBOOT_DELAY)
        deadline = time.monotonic() + JETSON_REBOOT_WINDOW
        waited = False
        while time.monotonic() < deadline:
            with self._lock:
                pending = self._jetson_reboot_at is not None
            if not pending:
                break
            waited = True
            time.sleep(0.2)
        if waited:
            time.sleep(1.0)
        subprocess.run(REBOOT_CMD, timeout=30, check=False)

    def _jetson_host_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._jetson_host is None:
            return None
        age = now - self._jetson_host_stamp
        state = dict(self._jetson_host)
        state['alive'] = age < JETSON_STATUS_TIMEOUT
        state['age_s'] = round(age, 1)
        state['reboot_pending'] = self._jetson_reboot_at is not None
        state['upgrade_pending'] = self._jetson_upgrade_at is not None
        return state

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

    def neutralize(self):
        """手動指令をニュートラルにする。"""
        return self.publish_cmd_vel(0.0, 0.0, 0.0)

    def stop(self):
        """全モードで停止を要求する。"""
        cmd = self.neutralize()
        if self.mode_request_pub is not None:
            self._publish_mode_request('stop')
        cmd['mode'] = 'stop'
        return cmd

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
                'auth_required': bool(self.api_token),
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
                'drive_mode': self._drive_mode_state(),
                'autonomy': self._autonomy_state(),
                'cliff': self._cliff_state(),
                'slam': self._slam_state(),
                'jetson_host': self._jetson_host_state(),
                'joy': {
                    'connected': (self.get_clock().now().nanoseconds * 1e-9
                                  - self._joy_stamp) < self.joy_timeout,
                    'active': self._joy_moving,
                    'x': round(self._joy_cmd[0], 2),
                    'y': round(self._joy_cmd[1], 2),
                    'z': round(self._joy_cmd[2], 2),
                },
                'limits': {
                    'max_linear_speed': self.max_linear,
                    'max_angular_speed': self.max_angular,
                },
            }

    def architecture(self):
        params = {}
        try:
            with open(self.params_config, encoding='utf-8') as f:
                config = yaml.safe_load(f) or {}
            params = {
                node: (section.get('ros__parameters') or {})
                for node, section in config.items()
                if isinstance(section, dict)
            }
        except (OSError, yaml.YAMLError, AttributeError):
            pass
        params.setdefault('dashboard_node', {})['api_token'] = bool(self.api_token)
        data = load_architecture(self.architecture_config, params)
        try:
            topics = []
            for name, _ in self.get_topic_names_and_types():
                if self.get_publishers_info_by_topic(name):
                    topics.append(name)
            data['live'] = {
                'topics': sorted(topics),
                'nodes': sorted(self.get_node_names()),
            }
        except Exception:
            data['live'] = {'topics': [], 'nodes': []}
        return data


def create_app(node: DashboardNode) -> FastAPI:
    """ダッシュボードの FastAPI アプリを生成する。"""
    share_dir = get_package_share_directory('ai_car_web')
    static_dir = os.path.join(share_dir, 'static')
    print3d_dir = os.path.join(share_dir, 'print3d')
    app = FastAPI(title='AI-CAR Dashboard')

    @app.middleware('http')
    async def revalidate(request: Request, call_next):
        response = await call_next(request)
        if request.url.path == '/' or request.url.path.startswith('/print3d/'):
            response.headers['Cache-Control'] = 'no-cache'
        return response

    def print3d_rev():
        try:
            return int(max(os.path.getmtime(os.path.join(print3d_dir, f))
                           for f in os.listdir(print3d_dir)))
        except (OSError, ValueError):
            return 0

    def require_token(request: Request):
        if not node.api_token:
            return
        auth = request.headers.get('authorization', '')
        token = (auth[7:] if auth.lower().startswith('bearer ')
                 else request.headers.get('x-api-token', ''))
        if not hmac.compare_digest(token, node.api_token):
            raise HTTPException(status_code=401, detail='API トークンが必要です')

    @app.get('/')
    def index():
        return FileResponse(os.path.join(static_dir, 'index.html'))

    @app.get('/api/status')
    def status():
        return node.telemetry()

    @app.get('/api/gpio')
    def gpio():
        return build_pinout(node.gpio_config)

    @app.get('/api/motor_hat')
    def motor_hat():
        return load_motor_hat(node.motor_hat_config)

    @app.get('/api/architecture')
    def architecture():
        data = node.architecture()
        if isinstance(data.get('print3d'), dict):
            data['print3d']['rev'] = print3d_rev()
        return data

    @app.get('/api/dev_diary')
    def dev_diary():
        return load_dev_diary(node.repo_dir)

    @app.post('/api/cmd_vel', dependencies=[Depends(require_token)])
    def cmd_vel(req: CmdVelRequest):
        return node.publish_cmd_vel(req.linear_x, req.linear_y, req.angular_z)

    @app.post('/api/stop', dependencies=[Depends(require_token)])
    def stop():
        return node.stop()

    @app.post('/api/jetson/detections', dependencies=[Depends(require_token)])
    def jetson_detections(req: JetsonDetectionsRequest):
        return node.publish_jetson_detections(req)

    @app.post('/api/jetson/status', dependencies=[Depends(require_token)])
    def jetson_status(req: JetsonStatusRequest):
        return node.update_jetson_status(req)

    @app.post('/api/jetson/reboot', dependencies=[Depends(require_token)])
    def jetson_reboot():
        return node.request_jetson_reboot()

    @app.post('/api/jetson/upgrade', dependencies=[Depends(require_token)])
    def jetson_upgrade():
        return node.request_jetson_upgrade()

    @app.post('/api/system/reboot', dependencies=[Depends(require_token)])
    def system_reboot():
        return node.reboot_pi()

    @app.post('/api/system/upgrade', dependencies=[Depends(require_token)])
    def system_upgrade():
        return node.upgrade_pi()

    @app.get('/api/drive_mode')
    def drive_mode():
        return node.telemetry()['drive_mode'] or {'mode': None, 'alive': False}

    @app.post('/api/drive_mode', dependencies=[Depends(require_token)])
    def set_drive_mode(req: DriveModeRequest):
        return node.request_drive_mode(req.mode)

    @app.get('/api/slam/map.png')
    def slam_map():
        png = node.map_png()
        if png is None:
            return Response(status_code=503)
        return Response(content=png, media_type='image/png', headers={'Cache-Control': 'no-store'})

    @app.post('/api/slam/save', dependencies=[Depends(require_token)])
    def slam_save():
        return node.save_map()

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
    if os.path.isdir(print3d_dir):
        app.mount('/print3d', StaticFiles(directory=print3d_dir, follow_symlink=True),
                  name='print3d')
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
