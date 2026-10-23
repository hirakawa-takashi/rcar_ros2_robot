"""AI-CAR Webダッシュボードノード。

FastAPI サーバーを別スレッドで起動し、ブラウザ・ゲームパッドからの手動操作コマンドを
手動指令トピック（既定 /cmd_vel_manual。drive_mode_node がモードに応じて /cmd_vel へ中継）
へ publish し、センサートピックのテレメトリを WebSocket で配信する。
"""

import asyncio
import hmac
import json
import math
import glob
import os
import re
import struct
import subprocess
import threading
import time
import unicodedata
import urllib.error
import urllib.parse
import urllib.request
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
from sensor_msgs.msg import CompressedImage, Imu, LaserScan, MagneticField
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

try:
    from slam_toolbox.srv import Reset as SlamReset
    from slam_toolbox.srv import DeserializePoseGraph, SerializePoseGraph
except ImportError:
    SlamReset = DeserializePoseGraph = SerializePoseGraph = None

from ai_car_web.architecture import VIDEO_DIR, load_architecture, read_program
from ai_car_web.dev_diary import find_repo, query_dev_diary, sync_dev_diary
from ai_car_web.version import load_version
from ai_car_web.gpio_pinout import build_pinout
from ai_car_web.motor_hat import load_motor_hat
from ai_car_web.patrol import coverage_points, frontier_goal, new_objects, order_points

MAX_SCAN_POINTS = 720


class CmdVelRequest(BaseModel):
    """ブラウザから受け取る速度指令。"""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class DriveOutputRequest(BaseModel):
    """ダッシュボードの「出力」（10〜100 %）。手動・ゲームパッド・自動の最大の速さ。"""

    percent: float


class DriveModeRequest(BaseModel):
    """ブラウザから受け取る運転モード切替要求（manual / auto / stop）。"""

    mode: str


class CliffMarkRequest(BaseModel):
    """地図の上の段差の印（map 座標）。add は置く、remove はその 0.3 m 以内を消す。"""

    action: str
    x: float
    y: float


class SlamSaveRequest(BaseModel):
    """地図を保存するときの名前（同じ名前があれば上書き）。空なら日時。"""

    name: str = ''


class SlamMapRequest(BaseModel):
    """保存した地図の id（slam_YYYYmmdd_HHMMSS）。rename のときは新しい名前も。"""

    id: str
    name: str = ''


class PlaceRequest(BaseModel):
    """地図の緑の点（名前つきの場所、map 座標）。add は置く（here なら今の車の所。同じ名前は動かす）、remove は消す、rename は new_name へ。"""

    action: str
    name: str = ''
    x: float | None = None
    y: float | None = None
    here: bool = False
    new_name: str = ''


class MappingRequest(BaseModel):
    """地図作り。go は今の地図を消し、止まって北をはかってから、まだ見ていない所（はし）を Nav2 で自動で回る。
    cancel はやめる。north は北の向きを deg 度直す（スマホのコンパスに合わせる）。"""

    action: str = 'go'
    deg: float = 0.0


class HomeRequest(BaseModel):
    """ホームポジション。set は今の車の位置と向き（x, y があればその所）をホームにする、go はホームへもどる、cancel はやめる、clear は消す。"""

    action: str = 'go'
    x: float | None = None
    y: float | None = None
    yaw_deg: float | None = None


class PatrolRequest(BaseModel):
    """地図の全部の「見る所」と緑の点を回って、異常（人・床の物・新しい段差）を見る（go）。cancel はやめて止まる。"""

    action: str = 'go'


class RouteRequest(BaseModel):
    """緑の点を names の順に Nav2 で回る（go）。cancel はやめて止まる。"""

    action: str = 'go'
    names: list[str] = []


class NavRequest(BaseModel):
    """地図の行き先（map 座標）。plan は道だけ、go は自動にして向かう、cancel はやめる。"""

    action: str
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0


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
    stt_engine: str = ''
    tts_engine: str = ''
    llm_size_mb: float | None = None
    llm_vram_mb: float | None = None
    updates_pending: int | None = None
    updates_held: int | None = None
    security_pending: int | None = None
    reboot_required: bool | None = None
    last_upgrade: float | None = None
    ips: list[dict[str, str]] = []
    can_reboot: bool | None = None
    can_poweroff: bool | None = None
    can_upgrade: bool | None = None
    upgrade_running: bool | None = None
    upgrade_result: str = ''
    upgrade_percent: int | None = None
    upgrade_phase: str = ''


class MyPCStatusRequest(JetsonStatusRequest):
    """パソコン（jetson/stackchan_bridge/pc/pc_status.py）から受け取る状態。Jetson と同じ形に GPU の名前を足す。"""

    gpu_name: str = ''
    gpu_driver: str = ''
    ubuntu_updates_pending: int | None = None
    ubuntu_security_pending: int | None = None
    win_updates_pending: int | None = None
    win_security_pending: int | None = None
    win_reboot_required: bool | None = None


def host_status(req: JetsonStatusRequest):
    """Jetson・パソコンから届いた状態の、文字の長さと項目の数を絞る。"""
    status = req.model_dump()
    for key, value in status.items():
        if isinstance(value, str):
            status[key] = value[:64]
    status['temperatures_c'] = {k[:16]: round(v, 1)
                                for k, v in list(req.temperatures_c.items())[:8]}
    status['services'] = {k[:16]: v for k, v in list(req.services.items())[:8]}
    status['llm_models'] = [m[:64] for m in req.llm_models[:4]]
    status['ips'] = [{'iface': str(ip.get('iface', ''))[:16], 'addr': str(ip.get('addr', ''))[:64]}
                     for ip in req.ips[:8]]
    return status


JETSON_STATUS_TIMEOUT = 5.0
MYPC_ACTIONS = ('reboot', 'upgrade')
JETSON_REBOOT_WINDOW = 10.0
PI_REBOOT_DELAY = 10.0
STACKCHAN_BRIDGE_PORT = 8003
STACKCHAN_TIMEOUT = 2.0
STACKCHAN_PHOTO_TIMEOUT = 20.0
STACKCHAN_MODE_TIMEOUT = 45.0  # 「ネット」にするとき、ネットの XiaoZhi につなぐのを待つ秒
# 名前を付けて保存した地図の id（ファイル名の元）
SLAM_ID_RE = re.compile(r'slam_\d{8}_\d{6}')
PLACE_MAX = 30           # 地図 1 つの緑の点（名前つきの場所）の数
PLACE_NEAR_M = 0.3       # この近さの点は同じ場所とみなす
ROUTE_PAUSE_S = 2.0      # 点に着いたら、ここだけ止まってから次へ
ROUTE_LEG_TIMEOUT_S = 180.0  # 1 つの点にこれより長くかかったらやめる
ROUTE_START_WAIT_S = 10.0    # Nav2 がこの間に受け付けなければやめる
ROUTE_FAIL = {'aborted', 'no_path', 'unavailable', 'canceled'}
ROUTE_RETRIES = 3        # 行けなかった点に、まわり道をさがして向かいなおす回数
ROUTE_RETRY_WAIT_S = 2.0  # 向かいなおす前に止まって待つ（LiDAR の地図に、ふさいでいる物を入れる）
# Nav2（ComputePathToPose）の「道がない」エラー。行き先が地図の外・ふさがっている・道がない。まわり道もない
NAV_NO_DETOUR = {204, 206, 208}
PATROL_SPACING_M = 1.5    # 地図をこの升に分けて、升ごとに 1 つ「見る所」を置く
PATROL_CLEAR_M = 0.25     # 壁・不明からこれより近い所には置かない。0.35 m では幅 70 cm のドア口・1 m の通路を通れず、となりの部屋に置かなかった
PATROL_MAX = 25           # 見る所（緑の点をふくむ）の上限。多ければ升を大きくする
PATROL_NEAR_GREEN_M = 0.75  # 緑の点にこれより近い「見る所」は置かない（緑の点で見る）
MAP_NORTH_S = 10.0       # 地図作りの始めに、止まったまま北をはかる秒数
MAP_WAIT_S = 20.0        # そのあと地図と車の位置が来るまで待つ秒数
MAP_DWELL_S = 2.0        # はしに着いたら止まって北をはかり、次をえらぶ
MAP_LIMIT_S = 900.0      # 15 分でやめる（のこり 1 分でスタートにもどりはじめる）
MAP_MAX_GOALS = 40
LIDAR_WINDOW_S = 3.0     # LiDAR のよごれチェック: この秒数のスキャンで見る
LIDAR_MIN_RATIO = 0.65   # 返る点がこれより少ない（家の中ならほぼ全部返る）か、
LIDAR_FAR_M = 2.5        # これより遠くが 1 点も見えないと「レンズがよごれている」とみる（よごれ 1.4〜1.8 m・きれい 3.8 m）
LIDAR_WARN_S = 30.0      # 走っているあいだ、この秒数続けて悪いと知らせる（止めはしない）
MAP_CLEAR_M = 0.20       # 「はし」をえらぶとき壁からはなす [m]。0.30 だと 70 cm のドア口を通れない（車の幅 33 cm）
MAP_CLIFF_M = 0.5        # 段差の印から、この半径の中は「はし」をえらぶ道に入れない [m]
MAP_SPEED_PERCENT = 10.0  # 地図作りのあいだの「出力」の上限
MAP_HOME_SPEED_PERCENT = 20.0  # 地図作りの最後に、スタートへもどるときの「出力」
PATROL_LIMIT_S = 900.0    # 15 分でやめる
PATROL_LEG_TIMEOUT_S = 120.0  # 1 か所にこの秒数で着かなければ、とばして次へ
PATROL_DWELL_S = 1.5      # 向きを変えたあと止まって待ってから写真・検出を見る（ぶれ・検出のおくれ）
PATROL_LOOKS = 4          # 1 か所で 90° ずつ 4 回見る
PATROL_KEEP = 10          # 残すパトロールの結果の数
PATROL_ID_RE = re.compile(r'\d{8}-\d{6}')
PERSON_LABELS = {'人'}
LIDAR_OFFSET = (0.043, 0.008)  # base_footprint から見た LiDAR の位置 [前, 左] m


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def place_key(name):
    """場所の名前をくらべる形（空白・記号を消し、全角半角をそろえ、カタカナをひらがなにする）。"""
    t = unicodedata.normalize('NFKC', name or '')
    t = re.sub(r'[\s、。,.・「」!！?？]', '', t)
    return ''.join(chr(ord(c) - 0x60) if 'ァ' <= c <= 'ヶ' else c for c in t).lower()


def clean_places(items):
    """保存した緑の点を [{name, x, y}] にそろえる（おかしいものは捨てる）。"""
    out = []
    for it in items or []:
        try:
            name = str(it['name']).strip()[:20]
            x, y = round(float(it['x']), 2), round(float(it['y']), 2)
        except (KeyError, TypeError, ValueError):
            continue
        if name and math.isfinite(x) and math.isfinite(y) and all(place_key(o['name']) != place_key(name) for o in out):
            out.append({'name': name, 'x': x, 'y': y})
    return out[:PLACE_MAX]


def clean_home(it):
    """保存したホームを {x, y, yaw_deg} にそろえる（おかしければ None）。"""
    try:
        x, y, yaw = round(float(it['x']), 2), round(float(it['y']), 2), round(float(it['yaw_deg']), 1)
    except (KeyError, TypeError, ValueError):
        return None
    return {'x': x, 'y': y, 'yaw_deg': yaw} if all(map(math.isfinite, (x, y, yaw))) else None
REBOOT_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'reboot']
POWEROFF_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'poweroff']
UPGRADE_CMD = ['sudo', '-n', '/usr/bin/systemctl', 'start', '--no-block', 'ai-car-upgrade.service']


def _can_sudo(cmd):
    """sudoers でパスワードなしに cmd を実行できるか（sudo -n -l の NOPASSWD の行で見る）。"""
    try:
        out = subprocess.run(['sudo', '-n', '-l'], capture_output=True, text=True,
                             timeout=5, check=False).stdout
    except (OSError, subprocess.TimeoutExpired):
        return False
    want = ' '.join(cmd[2:])
    return any('NOPASSWD:' in line and line.rstrip().endswith(want) for line in out.splitlines())


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


def _map_server_table():
    """OccupancyGrid の値 → map_server の trinary の .pgm（占有 ≥ 65 は 0、空き ≤ 19 は 254、ほかは 205）。"""
    table = bytearray(256)
    for u in range(256):
        table[u] = 205 if u > 100 else 0 if u >= 65 else 254 if u <= 19 else 205
    return bytes(table)


MAP_SERVER_TABLE = _map_server_table()


def compose_map_pose(pose, odom_at, odom_now):
    """/pose（map）に、そのときから今までの /odom の動きを足した map の位置。"""
    x, y, yaw = pose['x'], pose['y'], math.radians(pose['yaw_deg'])
    if odom_at and odom_now:
        dyaw = yaw - math.radians(odom_at['yaw_deg'])
        dx, dy = odom_now['x'] - odom_at['x'], odom_now['y'] - odom_at['y']
        c, s = math.cos(dyaw), math.sin(dyaw)
        x, y = x + c * dx - s * dy, y + s * dx + c * dy
        yaw += math.radians(odom_now['yaw_deg'] - odom_at['yaw_deg'])
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    return {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(math.degrees(yaw), 2)}


def write_map_files(name, grid):
    """地図を map_server の形（name.pgm と name.yaml）で書く。grid は _map_cb で取っておいたもの。"""
    width, height = grid['width'], grid['height']
    pixels = grid['data'].translate(MAP_SERVER_TABLE)
    rows = b''.join(pixels[r * width:(r + 1) * width] for r in range(height - 1, -1, -1))
    with open(name + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (width, height) + rows)
    with open(name + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(name)}.pgm\n'
                'mode: trinary\n'
                f'resolution: {grid["resolution"]:.3f}\n'
                f'origin: [{grid["origin_x"]:.3f}, {grid["origin_y"]:.3f}, {grid["origin_yaw"]:.4f}]\n'
                'negate: 0\n'
                'occupied_thresh: 0.65\n'
                'free_thresh: 0.196\n')


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
        self.declare_parameter('mag_topic', '/imu/mag')
        self.declare_parameter('mag_status_topic', '/imu/mag_status')
        self.declare_parameter('mag_calibrate_service', '/imu/mag_calibrate')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('system_status_topic', '/system_status')
        self.declare_parameter('camera_topic', '/camera/image_raw/compressed')
        self.declare_parameter('camera_stream_rate', 10.0)
        self.declare_parameter('obstacle_topic', '/obstacle_status')
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('cmd_timeout', 0.7)
        # 「出力」の初期値（%）。手動・ゲームパッド・自動の最大の速さ = max_* × 出力
        self.declare_parameter('output_percent', 15.0)
        self.declare_parameter('drive_output_topic', '/drive_output')
        self.declare_parameter('telemetry_rate', 5.0)
        # LiDAR 点群をブラウザに送るときの上限点数（間引き）
        self.declare_parameter('scan_max_points', MAX_SCAN_POINTS)
        self.declare_parameter('scan_angle_offset_deg', 0.0)
        self.declare_parameter('gpio_config', '')
        self.declare_parameter('motor_hat_config', '')
        self.declare_parameter('architecture_config', '')
        self.declare_parameter('params_config', '')
        self.declare_parameter('repo_dir', '')
        self.declare_parameter('diary_db', '~/.ros/ai_car_dev_diary.sqlite')
        # ゲームパッド（joy_teleop_node）からの正規化指令。空で無効
        self.declare_parameter('joy_cmd_topic', '/joy_cmd')
        self.declare_parameter('joy_timeout', 1.0)
        # 運転モード（drive_mode_node）。空で無効
        self.declare_parameter('drive_mode_topic', '/drive_mode')
        self.declare_parameter('drive_mode_request_topic', '/drive_mode_request')
        # 地図の行き先（autonomy_node の Level 2）。空で無効
        self.declare_parameter('nav_goal_topic', '/nav_goal')
        self.declare_parameter('api_token', '')
        # 自律走行の状態（autonomy_node）。空で無効
        self.declare_parameter('autonomy_status_topic', '/autonomy_status')
        # 落下防止センサー（cliff_node）。空で無効
        self.declare_parameter('cliff_topic', '/cliff_status')
        # SLAM（slam_toolbox）の地図と自己位置。空で無効
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('slam_pose_topic', '/pose')
        # 地図クリアで呼ぶ slam_toolbox のサービス。空で無効
        self.declare_parameter('slam_reset_service', '/slam_toolbox/reset')
        # 磁気の北と本当の北のずれ（東がプラス）。熊本は西へ約 7.3°
        self.declare_parameter('north_declination_deg', -7.3)
        self.north_declination = float(self.get_parameter('north_declination_deg').value)
        # スタックちゃんのカードに出す会話の様子（Jetson の stackchan_bridge）。空なら Jetson が送ってくる IP アドレスの 8003
        self.declare_parameter('stackchan_bridge_url', '')
        # 地図の保存先（空なら ~/maps）
        self.declare_parameter('map_save_dir', '')
        # パトロールの結果（report.json と写真）の保存先（空なら ~/patrols）
        self.declare_parameter('patrol_dir', '')
        # 「地図を保存」で slam_toolbox の地図データ（pose graph）も保存し、起動したときに読みこむ。空で無効
        self.declare_parameter('slam_serialize_service', '/slam_toolbox/serialize_map')
        self.declare_parameter('slam_deserialize_service', '/slam_toolbox/deserialize_map')
        self.declare_parameter('slam_load_on_start', False)
        # Jetson から POST /api/jetson/detections で受けた検出を流すトピック。空で無効
        self.declare_parameter('jetson_detections_topic', '/jetson_detections')

        self.host = self.get_parameter('host').value
        self.port = int(self.get_parameter('port').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self._output = self._clamp_output(float(self.get_parameter('output_percent').value) / 100.0)
        self.telemetry_rate = float(self.get_parameter('telemetry_rate').value)
        self.scan_max_points = max(1, int(self.get_parameter('scan_max_points').value))
        self.camera_stream_rate = float(self.get_parameter('camera_stream_rate').value)
        self.api_token = str(self.get_parameter('api_token').value) or os.environ.get(
            'AI_CAR_API_TOKEN', '')
        self.stackchan_bridge_url = str(self.get_parameter('stackchan_bridge_url').value).rstrip('/')
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
        self.diary_db = self.get_parameter('diary_db').value
        if self.repo_dir:
            threading.Thread(target=self._sync_diary, daemon=True).start()
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
        self.output_pub = self.create_publisher(
            Float32, self.get_parameter('drive_output_topic').value,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.output_pub.publish(Float32(data=self._output))
        self.status_pub = self.create_publisher(String, '~/status', 10)
        jetson_topic = self.get_parameter('jetson_detections_topic').value
        self.jetson_pub = (self.create_publisher(String, jetson_topic, 10)
                           if jetson_topic else None)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._scan_cb, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter('imu_topic').value, self._imu_cb, sensor_qos)
        mag_topic = str(self.get_parameter('mag_topic').value)
        if mag_topic:
            self.create_subscription(MagneticField, mag_topic, self._mag_cb, sensor_qos)
        mag_status_topic = str(self.get_parameter('mag_status_topic').value)
        if mag_status_topic:
            self.create_subscription(String, mag_status_topic, self._mag_status_cb, 10)
        mag_calibrate_service = str(self.get_parameter('mag_calibrate_service').value)
        self.mag_calibrate_client = (self.create_client(Trigger, mag_calibrate_service)
                                     if mag_calibrate_service else None)
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
        nav_goal_topic = self.get_parameter('nav_goal_topic').value
        self.nav_goal_pub = (self.create_publisher(String, nav_goal_topic, 10)
                             if nav_goal_topic else None)
        self.declare_parameter('cliff_clear_topic', '/cliff_marks_clear')
        cliff_clear_topic = self.get_parameter('cliff_clear_topic').value
        self.cliff_clear_pub = (self.create_publisher(String, cliff_clear_topic, 10)
                                if cliff_clear_topic else None)
        self.declare_parameter('cliff_edit_topic', '/cliff_marks_edit')
        cliff_edit_topic = self.get_parameter('cliff_edit_topic').value
        self.cliff_edit_pub = (self.create_publisher(String, cliff_edit_topic, 10)
                               if cliff_edit_topic else None)
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
        slam_reset_service = self.get_parameter('slam_reset_service').value
        self.slam_reset_client = (self.create_client(SlamReset, slam_reset_service)
                                  if slam_reset_service and SlamReset is not None else None)
        self.patrol_dir = self.get_parameter('patrol_dir').value or os.path.join(
            os.path.expanduser('~'), 'patrols')
        self.map_save_dir = self.get_parameter('map_save_dir').value or os.path.join(
            os.path.expanduser('~'), 'maps')
        srv = self.get_parameter('slam_serialize_service').value
        self.slam_serialize_client = (self.create_client(SerializePoseGraph, srv)
                                      if srv and SerializePoseGraph is not None else None)
        srv = self.get_parameter('slam_deserialize_service').value
        self.slam_deserialize_client = (self.create_client(DeserializePoseGraph, srv)
                                        if srv and DeserializePoseGraph is not None else None)
        self.slam_load_on_start = bool(self.get_parameter('slam_load_on_start').value)

        self._lock = threading.Lock()
        self._last_cmd = Twist()
        self._last_cmd_time = 0.0
        self._cmd_active = False
        self._scan = None
        self._lidar_hist = deque()
        self._lidar_bad_since = None
        self._imu = None
        self._imu_tilt = (0.0, 0.0)
        self._mag = None
        self._mag_status = None
        self._mag_status_stamp = 0.0
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
        self._mypc_host = None
        self._mypc_host_stamp = 0.0
        self._mypc_asked = {}
        self._jetson_reboot_at = None
        self._jetson_poweroff_at = None
        self._jetson_upgrade_at = None
        self._map = None
        self._map_rev = 0
        self._map_png = None
        self._map_grid = None
        self._map_stamp = 0.0
        self._slam_pose = None
        self._slam_pose_stamp = 0.0
        self._slam_pose_odom = None
        self._map_saved = None
        self._map_loaded = None
        self._slam_load_pending = self.slam_load_on_start and self.slam_deserialize_client is not None
        self._marks_to_restore, self._marks_tries = [], 0
        self._places = []
        self._home = None
        self._route = None
        self._north = None
        self._north_samples = []
        self._frame = None
        self._frame_stamp = 0.0
        self._frame_count = 0
        self._frame_bytes = 0
        self._frame_size = None
        self._frame_times = deque(maxlen=60)

        self.create_timer(0.1, self._watchdog_cb)
        self.create_timer(0.5, self._route_tick)
        if self.slam_deserialize_client is not None:
            self.create_timer(2.0, self._slam_load_cb)
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
            now = time.monotonic()
            self._lidar_hist.append((now, len(ranges) / max(1, len(msg.ranges)), max(ranges, default=0.0)))
            while self._lidar_hist and now - self._lidar_hist[0][0] > LIDAR_WINDOW_S:
                self._lidar_hist.popleft()
            h = self._lidar_health_locked(now)
            if h['ok']:
                self._lidar_bad_since = None
            elif self._lidar_bad_since is None:
                self._lidar_bad_since = now

    def _lidar_health_locked(self, now=None):
        """LiDAR の見え方（よごれチェック）。返る点の割合 [%] と、いちばん遠い点 [m]。_lock の中で呼ぶ。"""
        now = time.monotonic() if now is None else now
        hist = [h for h in self._lidar_hist if now - h[0] <= LIDAR_WINDOW_S]
        if not hist:
            return None
        ratio = sum(h[1] for h in hist) / len(hist)
        far = max(h[2] for h in hist)
        bad = self._lidar_bad_since
        return {'ratio': round(ratio * 100), 'far_m': round(far, 2),
                'ok': ratio >= LIDAR_MIN_RATIO and far >= LIDAR_FAR_M,
                'bad_s': round(now - bad, 1) if bad is not None else 0.0,
                'min_ratio': round(LIDAR_MIN_RATIO * 100), 'far_need_m': LIDAR_FAR_M}

    def _lidar_error(self):
        """走り出す前のチェック。よければ ''。"""
        with self._lock:
            h = self._lidar_health_locked()
        if h is None:
            return 'LiDAR のデータが来ていません'
        if h['ok']:
            return ''
        return (f"LiDAR が遠くまで見えません（返る点 {h['ratio']}%・いちばん遠い点 {h['far_m']} m）。"
                'LiDAR のレンズ（回る所の 2 つの小さな窓）をふいてください')

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
            self._imu_tilt = (roll, pitch)

    def _mag_cb(self, msg: MagneticField):
        m = msg.magnetic_field
        x, y, z = m.x * 1e6, m.y * 1e6, m.z * 1e6
        with self._lock:
            roll, pitch = self._imu_tilt
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        y1 = y * cr - z * sr
        z1 = y * sr + z * cr
        level_x = x * cp + z1 * sp
        heading = math.degrees(math.atan2(y1, level_x)) % 360.0
        with self._lock:
            self._mag = {
                'stamp': self._stamp_to_sec(msg.header.stamp),
                'x_ut': round(x, 1),
                'y_ut': round(y, 1),
                'z_ut': round(z, 1),
                'strength_ut': round(math.sqrt(x * x + y * y + z * z), 1),
                'heading_deg': round(heading, 1),
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
            self._map_rev += 1
            self._map = {
                'rev': self._map_rev,
                'width': info.width,
                'height': info.height,
                'resolution': round(info.resolution, 4),
                'origin_x': round(info.origin.position.x, 3),
                'origin_y': round(info.origin.position.y, 3),
                'origin_yaw_deg': round(math.degrees(yaw), 2),
                'frame_id': msg.header.frame_id,
            }
            self._map_png = png
            self._map_grid = {
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin_x': info.origin.position.x,
                'origin_y': info.origin.position.y,
                'origin_yaw': yaw,
                'data': bytes(msg.data),
            }
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
            self._slam_pose_odom = self._odom

    def _slam_state(self):
        now = time.time()
        map_age = now - self._map_stamp if self._map_stamp else None
        pose_age = now - self._slam_pose_stamp if self._slam_pose_stamp else None
        return {
            'alive': ((pose_age is not None and pose_age < 5.0)
                      or (map_age is not None and map_age < 5.0)),
            'map': self._map,
            'map_age': round(map_age, 1) if map_age is not None else None,
            'pose': self._slam_pose,
            'pose_age': round(pose_age, 1) if pose_age is not None else None,
            'can_save': self._map_grid is not None,
            'saved': self._map_saved,
            'loaded': self._map_loaded,
            'places': self._places,
            'home': self._home,
            'north': self._north,
        }

    def _map_pose_now(self):
        """今の車の位置（map）。/pose は地図にスキャンを足したときだけ来るので、そのあとの /odom の動きを足す。"""
        with self._lock:
            pose, at, now = self._slam_pose, self._slam_pose_odom, self._odom
        if pose is None:
            return None
        return compose_map_pose(pose, at, now)

    def _slam_files(self):
        return os.path.join(self.map_save_dir, 'slam_latest.json')

    def _map_base(self, map_id):
        if not isinstance(map_id, str) or not SLAM_ID_RE.fullmatch(map_id):
            raise HTTPException(status_code=400, detail='地図の id がおかしい')
        return os.path.join(self.map_save_dir, map_id)

    def _map_info(self, map_id):
        """保存した地図 1 つの情報（<id>.json）。なければ 404。"""
        base = self._map_base(map_id)
        try:
            with open(base + '.json', encoding='utf-8') as f:
                info = json.load(f)
        except (OSError, ValueError):
            raise HTTPException(status_code=404, detail='その地図はありません')
        info['id'] = map_id
        info['base'] = base
        return info

    def list_maps(self):
        """名前を付けて保存した地図（新しい順）。地図の絵・写真があるか、今使っているかも。"""
        maps = []
        for path in glob.glob(os.path.join(self.map_save_dir, 'slam_*.json')):
            map_id = os.path.basename(path)[:-5]
            if not SLAM_ID_RE.fullmatch(map_id):
                continue
            try:
                info = self._map_info(map_id)
            except HTTPException:
                continue
            base = info['base']
            if not all(os.path.exists(base + ext) for ext in ('.posegraph', '.data')):
                continue
            maps.append({'id': map_id, 'name': info.get('name') or map_id,
                         'stamp': info.get('stamp'), 'pose': info.get('pose'),
                         'cliff_marks': info.get('cliff_marks') or [], 'map': info.get('map'),
                         'places': clean_places(info.get('places')), 'home': clean_home(info.get('home')),
                         'north': info.get('north'),
                         'png': os.path.exists(base + '.png'), 'photo': os.path.exists(base + '.jpg')})
        maps.sort(key=lambda m: m['stamp'] or 0, reverse=True)
        with self._lock:
            loaded = self._map_loaded
        return {'maps': maps, 'current': loaded.get('id') if loaded and loaded.get('ok') else None}

    def map_file(self, map_id, ext):
        path = self._map_base(map_id) + ext
        if not os.path.exists(path):
            raise HTTPException(status_code=404, detail='ありません')
        return path

    def _remove_map_files(self, base):
        for ext in ('.posegraph', '.data', '.json', '.png', '.jpg'):
            try:
                os.remove(base + ext)
            except OSError:
                pass

    def _write_latest(self, info):
        latest = self._slam_files()
        with open(latest + '.tmp', 'w', encoding='utf-8') as f:
            json.dump({k: info[k] for k in ('base', 'pose', 'cliff_marks', 'stamp') if k in info}, f)
        os.replace(latest + '.tmp', latest)

    def _save_slam_graph(self, pose, name=''):
        """slam_toolbox の地図データを、名前・始めの位置・段差の印・地図の絵・カメラの写真といっしょに保存する。"""
        client = self.slam_serialize_client
        if client is None:
            return {'ok': False, 'reason': '地図データの保存が無効'}
        if not client.service_is_ready():
            return {'ok': False, 'reason': 'slam_toolbox が応答しない'}
        if pose is None:
            return {'ok': False, 'reason': '地図の上の車の位置がまだ分からない'}
        map_id = time.strftime('slam_%Y%m%d_%H%M%S')
        base = os.path.join(self.map_save_dir, map_id)
        done = threading.Event()
        future = client.call_async(SerializePoseGraph.Request(filename=base))
        future.add_done_callback(lambda _: done.set())
        if not done.wait(20.0):
            return {'ok': False, 'reason': 'slam_toolbox から返事がない'}
        res = future.result()
        if res is None or res.result != SerializePoseGraph.Response.RESULT_SUCCESS:
            return {'ok': False, 'reason': 'slam_toolbox が地図データを書けなかった'}
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            marks = (self._autonomy or {}).get('cliff_marks') or []
            png, meta = self._map_png, dict(self._map or {})
            frame = self._frame if self._frame is not None and now - self._frame_stamp < 3.0 else None
            loaded = self._map_loaded
            places = list(self._places)
            home = self._home
            north = self._north
        name = (name or '').strip()[:30] or (loaded.get('name') if loaded and loaded.get('ok') else '') \
            or time.strftime('地図 %m/%d %H:%M')
        meta.pop('rev', None)
        info = {'name': name, 'pose': pose, 'cliff_marks': marks, 'places': places, 'home': home, 'north': north,
                'stamp': round(time.time(), 1), 'map': meta}
        if png:
            with open(base + '.png', 'wb') as f:
                f.write(png)
        if frame:
            with open(base + '.jpg', 'wb') as f:
                f.write(frame)
        with open(base + '.json', 'w', encoding='utf-8') as f:
            json.dump(info, f, ensure_ascii=False)
        info['base'] = base
        self._write_latest(info)
        replaced = [m['id'] for m in self.list_maps()['maps'] if m['name'] == name and m['id'] != map_id]
        for old in replaced:
            self._remove_map_files(self._map_base(old))
        with self._lock:
            self._map_loaded = {'ok': True, 'id': map_id, 'name': name, 'pose': pose, 'marks': len(marks),
                                'stamp': round(time.time(), 1), 'reason': '', 'saved': True}
        return {'ok': True, 'id': map_id, 'name': name, 'pose': pose, 'marks': len(marks),
                'replaced': len(replaced)}

    def _deserialize(self, info, on_done=None):
        """保存した地図データを、保存したときの車の位置から始めるように読みこみ、段差の印を入れかえる。"""
        base, pose = info['base'], info['pose']
        req = DeserializePoseGraph.Request(
            filename=base, match_type=DeserializePoseGraph.Request.START_AT_GIVEN_POSE)
        req.initial_pose.x = float(pose['x'])
        req.initial_pose.y = float(pose['y'])
        req.initial_pose.theta = math.radians(float(pose['yaw_deg']))
        map_id = os.path.basename(base)
        name = info.get('name') or map_id
        marks = info.get('cliff_marks') or []
        places = clean_places(info.get('places'))
        home = clean_home(info.get('home'))

        def done(future):
            ok = future.result() is not None
            if ok and self.cliff_edit_pub is not None:
                if self.cliff_clear_pub is not None:
                    self.cliff_clear_pub.publish(String(data='clear'))
                self._marks_to_restore = [(float(m[0]), float(m[1])) for m in marks]
                self._marks_tries = 0
            with self._lock:
                if ok:
                    self._places = places
                    self._home = home
                    self._north = info.get('north')
                    self._north_samples = []
                self._map_loaded = {'ok': ok, 'id': map_id, 'name': name, 'pose': pose, 'marks': len(marks),
                                    'stamp': round(time.time(), 1),
                                    'reason': '' if ok else 'slam_toolbox が読みこめなかった'}
            self.get_logger().info(
                f'保存した地図 {name}（{map_id}）を読みこんだ（始めの位置 x {pose["x"]:.2f} y {pose["y"]:.2f}、段差の印 {len(marks)}）'
                if ok else f'保存した地図 {name}（{map_id}）を読みこめなかった')
            if on_done is not None:
                on_done(ok)

        self.slam_deserialize_client.call_async(req).add_done_callback(done)

    def _slam_load_cb(self):
        """保存した段差の印を戻す。slam_load_on_start なら起動して 1 回だけ、最後に使った地図を読みこむ。"""
        if self._marks_to_restore:
            self._restore_marks()
            return
        if not self._slam_load_pending:
            return
        try:
            with open(self._slam_files()) as f:
                info = json.load(f)
            info['base'], info['pose']
            ok = all(os.path.exists(info['base'] + ext) for ext in ('.posegraph', '.data'))
            try:
                with open(info['base'] + '.json', encoding='utf-8') as f:
                    info.update(json.load(f))
            except (OSError, ValueError):
                pass
        except (OSError, ValueError, KeyError, TypeError):
            ok = False
        if not ok:
            self._slam_load_pending = False
            return
        if self._map_stamp == 0.0 or not self.slam_deserialize_client.service_is_ready():
            return
        self._slam_load_pending = False
        self._deserialize(info)

    def load_map(self, map_id):
        """選んだ地図を読みこむ（止まっているときだけ）。車は、その地図の始めの位置・向きに置いておく。"""
        info = self._map_info(map_id)
        if self._moving():
            raise HTTPException(status_code=409, detail='走行中は地図を変えられません（先に止めてください）')
        client = self.slam_deserialize_client
        if client is None or not client.service_is_ready():
            raise HTTPException(status_code=503, detail='slam_toolbox が応答しません（use_slam:=true）')
        if self._map_stamp == 0.0:
            raise HTTPException(status_code=503, detail='slam_toolbox の地図がまだ来ていません（少し待つ）')
        if not all(os.path.exists(info['base'] + ext) for ext in ('.posegraph', '.data')):
            raise HTTPException(status_code=404, detail='地図データ（.posegraph / .data）がありません')
        if self.nav_goal_pub is not None:
            self.nav_goal_pub.publish(String(data=json.dumps(
                {'action': 'cancel', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})))
        self._slam_load_pending = False
        done, result = threading.Event(), {}

        def finished(ok):
            result['ok'] = ok
            done.set()

        self._deserialize(info, finished)
        if not done.wait(30.0):
            return {'ok': False, 'reason': 'slam_toolbox から返事がない'}
        if result['ok']:
            self._write_latest(info)
        return {'ok': result['ok'], 'id': map_id, 'name': info.get('name') or map_id,
                'reason': '' if result['ok'] else 'slam_toolbox が読みこめなかった'}

    def rename_map(self, map_id, name):
        info = self._map_info(map_id)
        name = (name or '').strip()[:30]
        if not name:
            raise HTTPException(status_code=400, detail='名前が空です')
        if any(m['name'] == name and m['id'] != map_id for m in self.list_maps()['maps']):
            raise HTTPException(status_code=409, detail=f'「{name}」はもうあります')
        base = info.pop('base')
        info.pop('id', None)
        info['name'] = name
        with open(base + '.json.tmp', 'w', encoding='utf-8') as f:
            json.dump(info, f, ensure_ascii=False)
        os.replace(base + '.json.tmp', base + '.json')
        with self._lock:
            if self._map_loaded and self._map_loaded.get('id') == map_id:
                self._map_loaded = dict(self._map_loaded, name=name)
        return {'ok': True, 'id': map_id, 'name': name}

    def delete_map(self, map_id):
        info = self._map_info(map_id)
        self._remove_map_files(info['base'])
        return {'ok': True, 'id': map_id}

    def _restore_marks(self):
        """保存した段差の印を、autonomy_node の印に入るまで 2 秒ごとに送りなおす（起動直後は届かないことがある）。"""
        with self._lock:
            have = (self._autonomy or {}).get('cliff_marks') or []
        missing = [(x, y) for x, y in self._marks_to_restore
                   if not any(math.hypot(x - m[0], y - m[1]) < 0.15 for m in have)]
        if not missing or self._marks_tries >= 30:
            if missing:
                self.get_logger().warn(f'段差の印 {len(missing)} 個を autonomy_node に戻せなかった')
            self._marks_to_restore = []
            return
        self._marks_tries += 1
        for x, y in missing:
            self.cliff_edit_pub.publish(String(data=json.dumps({'action': 'add', 'x': x, 'y': y})))

    def map_png(self):
        with self._lock:
            return self._map_png

    def save_map(self, name=''):
        """最後に届いた /map を map_save_dir に map_server の形（.pgm / .yaml）で、地図データは名前を付けて保存する。"""
        with self._lock:
            grid = self._map_grid
        if grid is None:
            return {'ok': False, 'reason': '地図がまだ届いていない（use_slam:=true）'}
        files = os.path.join(self.map_save_dir, time.strftime('map_%Y%m%d_%H%M%S'))
        pose = self._map_pose_now()
        try:
            os.makedirs(self.map_save_dir, exist_ok=True)
            write_map_files(files, grid)
            ok, reason = True, ''
        except OSError as exc:
            ok, reason = False, f'保存に失敗（{exc}）'
        graph = {'ok': False, 'reason': ''}
        if ok:
            try:
                graph = self._save_slam_graph(pose, name)
            except OSError as exc:
                graph = {'ok': False, 'reason': f'地図データの保存に失敗（{exc}）'}
        saved = {'ok': ok, 'name': files, 'stamp': round(time.time(), 1), 'reason': reason, 'graph': graph}
        with self._lock:
            self._map_saved = saved
        return saved

    def clear_map(self):
        """slam_toolbox の地図を消して、今いる所から作り直す（行き先もやめる）。"""
        if self.slam_reset_client is None:
            return {'ok': False, 'reason': '地図クリアが無効です（slam_toolbox なし）'}
        if not self.slam_reset_client.service_is_ready():
            return {'ok': False, 'reason': 'slam_toolbox が応答しません（use_slam:=true）'}
        if self.nav_goal_pub is not None:
            msg = String()
            msg.data = json.dumps({'action': 'cancel', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
            self.nav_goal_pub.publish(msg)
        done = threading.Event()
        future = self.slam_reset_client.call_async(SlamReset.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0):
            return {'ok': False, 'reason': 'slam_toolbox から返事がない'}
        res = future.result()
        if res is None or res.result != SlamReset.Response.RESULT_SUCCESS:
            return {'ok': False, 'reason': 'slam_toolbox が地図を消せなかった'}
        if self.cliff_clear_pub is not None:
            self.cliff_clear_pub.publish(String(data='clear'))
        with self._lock:
            self._map = None
            self._map_png = None
            self._map_grid = None
            self._map_stamp = 0.0
            self._slam_pose = None
            self._slam_pose_stamp = 0.0
            self._map_saved = None
            self._map_loaded = {'ok': True, 'id': None, 'name': '', 'new': True, 'pose': None, 'marks': 0,
                                'stamp': round(time.time(), 1), 'reason': ''}
            self._places = []
            self._home = None
            self._route = None
            self._north = None
        self._north_samples = []
        self._marks_to_restore = []
        self.get_logger().info('SLAM の地図をクリア')
        return {'ok': True}

    def _obstacle_cb(self, msg: String):
        try:
            obstacle = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('obstacle_status の JSON を解析できません')
            return
        with self._lock:
            self._obstacle = obstacle
            self._obstacle_stamp = self.get_clock().now().nanoseconds * 1e-9

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

    def _mag_status_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._mag_status = payload
            self._mag_status_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _mag_status_state(self):
        if self._mag_status is None:
            return None
        state = dict(self._mag_status)
        state['alive'] = self.get_clock().now().nanoseconds * 1e-9 - self._mag_status_stamp < 3.0
        return state

    def start_mag_calibration(self):
        """imu_node に磁気の補正（床でまわしながら記録）を始めさせる。"""
        client = self.mag_calibrate_client
        if client is None or not client.service_is_ready():
            return {'ok': False, 'reason': 'imu_node が応答しません'}
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(3.0):
            return {'ok': False, 'reason': 'imu_node から返事がない'}
        res = future.result()
        if res is None or not res.success:
            return {'ok': False, 'reason': res.message if res else '失敗'}
        return {'ok': True, 'message': res.message}

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

    def edit_cliff_mark(self, req: CliffMarkRequest):
        """地図の上の段差の印を autonomy_node へ送る（置く・消す）。"""
        if req.action not in ('add', 'remove'):
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.cliff_edit_pub is None:
            return {'ok': False, 'error': '段差の印の編集が無効です'}
        self.cliff_edit_pub.publish(String(data=json.dumps({'action': req.action, 'x': req.x, 'y': req.y})))
        return {'ok': True}

    def _save_places(self):
        """緑の点を、今使っている保存した地図の <id>.json に書く（まだ保存していない地図なら「地図を保存」のときに書く）。"""
        with self._lock:
            loaded, places = self._map_loaded, list(self._places)
        map_id = loaded.get('id') if loaded and loaded.get('ok') else None
        if not map_id:
            return False
        try:
            info = self._map_info(map_id)
        except HTTPException:
            return False
        base = info.pop('base')
        info.pop('id', None)
        info['places'] = places
        with self._lock:
            info['home'] = self._home
            info['north'] = self._north
        with open(base + '.json.tmp', 'w', encoding='utf-8') as f:
            json.dump(info, f, ensure_ascii=False)
        os.replace(base + '.json.tmp', base + '.json')
        return True

    def find_place(self, name):
        key = place_key(name)
        with self._lock:
            return next((dict(p) for p in self._places if place_key(p['name']) == key), None) if key else None

    def edit_place(self, req: PlaceRequest):
        """地図の緑の点を置く・消す・名前を変える。保存した地図を使っていれば、その地図に書く。"""
        name = (req.name or '').strip()[:20]
        if req.action not in ('add', 'remove', 'rename'):
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if not place_key(name):
            return {'ok': False, 'error': '名前が空です'}
        if req.action == 'add':
            if req.here:
                pose = self._map_pose_now()
                if pose is None:
                    return {'ok': False, 'error': '地図の上の車の位置がまだ分かりません'}
                x, y = pose['x'], pose['y']
            elif req.x is None or req.y is None or not (math.isfinite(req.x) and math.isfinite(req.y)):
                return {'ok': False, 'error': 'x・y がありません'}
            else:
                x, y = req.x, req.y
            with self._lock:
                key = place_key(name)
                others = [p for p in self._places if place_key(p['name']) != key]
                if len(others) >= PLACE_MAX:
                    return {'ok': False, 'error': f'緑の点は {PLACE_MAX} 個までです'}
                place = {'name': name, 'x': round(float(x), 2), 'y': round(float(y), 2)}
                self._places = others + [place]
        else:
            with self._lock:
                old = next((p for p in self._places if place_key(p['name']) == place_key(name)), None)
                if old is None:
                    return {'ok': False, 'error': f'「{name}」という点はありません'}
                if req.action == 'remove':
                    self._places = [p for p in self._places if p is not old]
                    place = old
                else:
                    new = (req.new_name or '').strip()[:20]
                    if not place_key(new):
                        return {'ok': False, 'error': '新しい名前が空です'}
                    if any(p is not old and place_key(p['name']) == place_key(new) for p in self._places):
                        return {'ok': False, 'error': f'「{new}」はもうあります'}
                    place = dict(old, name=new)
                    self._places = [place if p is old else p for p in self._places]
        saved = self._save_places()
        self.get_logger().info(f'緑の点 {req.action}: {place["name"]}（x {place["x"]:.2f} y {place["y"]:.2f}）')
        return {'ok': True, 'place': place, 'saved': saved, 'places': self.places()['places']}

    def places(self):
        with self._lock:
            loaded = self._map_loaded
            return {'places': [dict(p) for p in self._places],
                    'map': loaded.get('name') if loaded and loaded.get('ok') else ''}

    def request_home(self, req: HomeRequest):
        """ホームポジションを決める（set）・消す（clear）・もどる（go）・やめる（cancel）。地図ごとに保存する。"""
        if req.action == 'set':
            pose = self._map_pose_now()
            if req.x is not None and req.y is not None:
                # 地図のばってん（×）の所。向きは指定がなければ今の車の向き
                yaw = req.yaw_deg if req.yaw_deg is not None else (pose or {}).get('yaw_deg', 0.0)
                pose = {'x': req.x, 'y': req.y, 'yaw_deg': yaw}
            if pose is None:
                return {'ok': False, 'error': '地図の上の車の位置がまだ分かりません'}
            home = clean_home(pose)
            if home is None:
                return {'ok': False, 'error': 'ホームの位置がおかしい'}
            with self._lock:
                self._home = home
            self.get_logger().info(f'ホーム: {home}')
            return {'ok': True, 'home': home, 'saved': self._save_places()}
        if req.action == 'clear':
            with self._lock:
                self._home = None
            return {'ok': True, 'home': None, 'saved': self._save_places()}
        if req.action == 'cancel':
            r = self._route
            if r is not None and r['state'] == 'running' and r.get('kind') == 'home':
                self._route_end('stopped', 'やめる指示')
            return {'ok': True}
        if req.action != 'go':
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.nav_goal_pub is None:
            return {'ok': False, 'error': '地図の行き先が無効です'}
        with self._lock:
            home = dict(self._home) if self._home else None
        if home is None:
            return {'ok': False, 'error': 'ホームがまだ決まっていません（「ここをホームにする」）'}
        auto = self._autonomy_state() or {}
        if not auto.get('alive') or not (auto.get('nav') or {}).get('available'):
            return {'ok': False, 'error': 'Nav2 が動いていません'}
        r = self._route
        if r is not None and r['state'] == 'running':
            self._route_end('stopped', 'ホームへもどる指示')
        stop = dict(home, name='ホーム', home=True)
        with self._lock:
            self._route = {'state': 'running', 'kind': 'home', 'stops': [stop], 'names': ['ホーム'],
                           'index': 0, 'arrived': [], 'reason': '', 'retries': 0, 'stamp': round(time.time(), 1),
                           'id': int(time.time() * 1000), '_sent': None, '_arrived_at': None}
        self.get_logger().info(f'ホームへもどる: {home}')
        self._route_send()
        res = self.request_drive_mode('auto')
        if not res.get('ok'):
            self._route_end('failed', res.get('error') or '自動にできない')
            return {'ok': False, 'error': res.get('error') or '自動にできない'}
        return {'ok': True, 'route': self._route_state()}

    def _route_state(self):
        r = self._route
        if r is None:
            return None
        return {k: v for k, v in r.items() if not k.startswith('_')}

    def request_route(self, req: RouteRequest):
        """緑の点を names の順に回る。知らない名前があれば走らない。行けない点があれば、そこで止まる。"""
        if req.action == 'cancel':
            with self._lock:
                running = self._route is not None and self._route['state'] == 'running'
            if running:
                self._route_end('stopped', 'やめる指示')
            return {'ok': True}
        if req.action != 'go':
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.nav_goal_pub is None:
            return {'ok': False, 'error': '地図の行き先が無効です'}
        if self._patrolling():
            return {'ok': False, 'error': 'パトロール中です。先にやめてください'}
        names = [n.strip() for n in req.names if place_key(n)][:20]
        if not names:
            return {'ok': False, 'error': '行く所がありません'}
        stops, unknown = [], []
        for n in names:
            p = self.find_place(n)
            if p is None:
                unknown.append(n)
            elif not stops or stops[-1]['name'] != p['name']:
                stops.append(p)
        if unknown:
            return {'ok': False, 'error': f'知らない場所: {"、".join(unknown)}', 'unknown': unknown}
        auto = self._autonomy_state() or {}
        if not auto.get('alive') or not (auto.get('nav') or {}).get('available'):
            return {'ok': False, 'error': 'Nav2 が動いていません'}
        with self._lock:
            self._route = {'state': 'running', 'stops': stops, 'names': [p['name'] for p in stops],
                           'index': 0, 'arrived': [], 'reason': '', 'retries': 0, 'stamp': round(time.time(), 1),
                           'id': int(time.time() * 1000), '_sent': None, '_arrived_at': None}
        self.get_logger().info(f'緑の点を回る: {" → ".join(p["name"] for p in stops)}')
        self._route_send()
        res = self.request_drive_mode('auto')
        if not res.get('ok'):
            self._route_end('failed', res.get('error') or '自動にできない')
            return {'ok': False, 'error': res.get('error') or '自動にできない'}
        return {'ok': True, 'route': self._route_state()}

    def _route_send(self, yaw=None):
        """今の点へ go を送る。yaw を渡せば、その向き（パトロールで同じ所で向きを変える）。"""
        r = self._route
        p = r['stops'][r['index']]
        if yaw is None and p.get('yaw_deg') is not None:
            yaw = p['yaw_deg']
        if yaw is None:
            # 着いたときの向きは、次の点の方（最後の点は来た向き）。点ごとにその場で回りなおさない
            nxt = r['stops'][r['index'] + 1] if r['index'] + 1 < len(r['stops']) else None
            frm = p if nxt else (r['stops'][r['index'] - 1] if r['index'] else self._map_pose_now())
            to = nxt or p
            yaw = math.degrees(math.atan2(to['y'] - frm['y'], to['x'] - frm['x'])) if frm and frm is not to else 0.0
        goal = {'x': p['x'], 'y': p['y'], 'yaw_deg': round(_wrap_deg(yaw), 1)}
        self.nav_goal_pub.publish(String(data=json.dumps(dict(goal, action='go'))))
        r['_goal'] = goal
        r['_sent'] = time.monotonic()
        r['_arrived_at'] = None
        r['target'] = p['name']

    def _route_end(self, state, reason=''):
        """回るのを終える。着いた・行けない・やめたのどれでも、止めて手動にもどす。"""
        with self._lock:
            r = self._route
            if r is None or r['state'] != 'running':
                return
            r['state'], r['reason'] = state, reason
            r['target'] = None
        if self.nav_goal_pub is not None and state != 'done':
            self.nav_goal_pub.publish(String(data=json.dumps(
                {'action': 'cancel', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})))
        self.neutralize()
        self.request_drive_mode('manual')
        self.get_logger().info(f'緑の点を回る: {state} {reason}')
        if r.get('kind') == 'patrol':
            self._patrol_finish(r)
        if r.get('kind') == 'explore':
            self.set_drive_output(r['_out0'] * 100.0)

    def _route_next(self, now):
        """今の点を終えて次へ（着いた）。最後なら終わる。"""
        r = self._route
        r['arrived'].append(r['stops'][r['index']]['name'])
        self._route_advance(now)

    def _route_advance(self, now):
        r = self._route
        r['retries'], r['look'], r['_dwell_at'] = 0, None, None
        r['index'] += 1
        if r['index'] >= len(r['stops']):
            self._route_end('done')
        else:
            r['_arrived_at'] = now

    def _route_fail(self, reason, now, again=True):
        """今の点に行けない。順に回るときは止まる。パトロールは、とばして次へ。

        地図作りは、とばして次の「はし」へ。again なら（段差でなければ）最後にもう一度だけ行ってみる。
        """
        r = self._route
        if r.get('kind') == 'explore':
            p = r['stops'][r['index']]
            if r.get('_returning'):
                self._route_end('done', f'スタートにもどれなかった（{reason}）')
                return
            r['skipped'].append(p['name'])
            r['_soft' if again and not r.get('_again') else '_bad'].append((p['x'], p['y']))
            self.get_logger().info(f'地図作り: とばす {reason}')
            self._explore_next(now)
            return
        if r.get('kind') != 'patrol':
            self._route_end('failed', reason)
            return
        r['skipped'].append(r['stops'][r['index']]['name'])
        self.get_logger().info(f'パトロール: とばす {reason}')
        self._route_advance(now)

    def _lidar_watch(self, r):
        """走っているあいだ LiDAR が LIDAR_WARN_S 秒続けて悪いと、route の lidar_warn に出す（止めはしない）。"""
        with self._lock:
            h = self._lidar_health_locked()
        if h is not None and h['bad_s'] >= LIDAR_WARN_S:
            if not r.get('lidar_warn'):
                r['lidar_warn'] = f"LiDAR が見えにくい（{h['ratio']}%・{h['far_m']} m）。レンズをふいてください"
                self.get_logger().warn(r['lidar_warn'])
        elif h is not None and h['ok']:
            r['lidar_warn'] = ''

    def _route_tick(self):
        r = self._route
        if r is None or r['state'] != 'running':
            return
        now = time.monotonic()
        self._lidar_watch(r)
        patrol = r.get('kind') == 'patrol'
        explore = r.get('kind') == 'explore'
        if explore and r.get('phase') in ('north', 'wait'):
            self._north_sample()
            if now - r['_start'] >= MAP_NORTH_S:
                r['phase'] = 'wait'
                self._explore_next(now)
            return
        if explore and r.get('_dwell_at') is not None:
            self._north_sample()
            if now - r['_dwell_at'] >= MAP_DWELL_S:
                r['_dwell_at'] = None
                self._explore_next(now)
            return
        dm = self._drive_mode_state() or {}
        if now - r['_sent'] > 3.0 and dm.get('mode') != 'auto':
            why = dm.get('guard') or dm.get('reason') or '自動を抜けた'
            self._route_end('stopped', str(why))
            return
        limit_s = PATROL_LIMIT_S if patrol else MAP_LIMIT_S
        if (patrol or explore) and now - r['_start'] > limit_s:
            self._route_end('timeout', f'{limit_s / 60:.0f} 分たった')
            return
        if r.get('_dwell_at') is not None:
            if now - r['_dwell_at'] >= PATROL_DWELL_S:
                r['_dwell_at'] = None
                self._patrol_look()
                r['look'] += 1
                if r['look'] < PATROL_LOOKS:
                    self._route_send(yaw=r['_look_yaw0'] + 360.0 / PATROL_LOOKS * r['look'])
                else:
                    self._route_next(now)
            return
        if r['_arrived_at'] is not None:
            if now - r['_arrived_at'] >= (ROUTE_RETRY_WAIT_S if r.get('_retry') else ROUTE_PAUSE_S):
                r['_retry'] = False
                self._route_send()
            return
        auto = self._autonomy_state() or {}
        nav = auto.get('nav') or {}
        p = r['stops'][r['index']]
        waited = now - r['_sent']
        if explore and not r.get('_returning'):
            # 段差センサーが反応した。段差の向こうは LiDAR では床に見えるので、この「はし」はやめてほかへ
            cliffs = len(auto.get('cliff_marks') or [])
            if cliffs > r.get('_cliffs', 0) or (str(auto.get('behavior') or '').startswith('cliff') and waited > 3.0):
                r['_cliffs'] = cliffs
                self._route_fail(f'{p["name"]}の前に段差', now, again=False)
                return
        g, sent = nav.get('goal') or {}, r.get('_goal') or {}
        mine = (abs((g.get('x') or 0) - sent.get('x', 0)) < 0.02 and abs((g.get('y') or 0) - sent.get('y', 0)) < 0.02
                and abs(_wrap_deg((g.get('yaw_deg') or 0) - sent.get('yaw_deg', 0))) < 1.0
                and auto.get('alive') and (nav.get('age_s') or 0) < waited + 0.5)
        state = nav.get('state') if mine else None
        if state == 'succeeded':
            if explore:
                r['arrived'].append(p['name'])
                r['_seen'].append((p['x'], p['y']))
                if r.get('_returning'):
                    self._route_end('done')
                else:
                    r['_dwell_at'] = now
                return
            if patrol and not p.get('home'):
                if r.get('look') is None:
                    r['look'], r['_look_yaw0'] = 0, sent.get('yaw_deg', 0.0)
                r['_dwell_at'] = now
                return
            self._route_next(now)
            return
        if state == 'canceled':
            self._route_end('stopped', nav.get('reason') or 'やめた')
            return
        if state == 'unavailable':
            self._route_end('failed', nav.get('reason') or 'Nav2 が動いていない')
            return
        if patrol and r.get('look') is not None and state in ROUTE_FAIL:
            # その場で向きを変えられなかった（壁が近いなど）。のこりの向きは見ないで次へ
            self._route_next(now)
            return
        if state == 'aborted':
            m = re.search(r'エラー (\d+)', nav.get('reason') or '')
            code = int(m.group(1)) if m else None
            if code in NAV_NO_DETOUR:
                self._route_fail(f'{p["name"]}に行けない（まわり道がない。Nav2 のエラー {code}）', now)
            elif r['retries'] >= ROUTE_RETRIES:
                self._route_fail(f'{p["name"]}に行けない（まわり道を {ROUTE_RETRIES} 回さがしても着かない。'
                                 f'{nav.get("reason") or ""}）', now)
            else:
                # ふさがっていた・進めなかった。止まって待ってから、今いる所から道を考えなおして向かう（まわり道）
                r['retries'] += 1
                r['_retry'] = True
                r['_arrived_at'] = now
                self.get_logger().info(f'緑の点: {p["name"]}にまわり道で向かいなおす（{r["retries"]} 回目、'
                                       f'{nav.get("reason") or ""}）')
            return
        if state in ROUTE_FAIL:
            self._route_fail(f'{p["name"]}に行けない（{nav.get("reason") or nav.get("label") or state}）', now)
            return
        if state is None and waited > ROUTE_START_WAIT_S:
            self._route_end('failed', 'Nav2 が受け付けない')
            return
        limit = PATROL_LEG_TIMEOUT_S if patrol or explore else ROUTE_LEG_TIMEOUT_S
        if waited > limit:
            self._route_fail(f'{p["name"]}まで {limit:.0f} 秒たっても着かない', now)

    # --- 地図作り（北・自動の探検） ---
    def _north_sample(self):
        """1 回: 地図の上で本当の北がどの向きか（車の向き＋磁気の向き＋偏角）をためる。止まっているときだけ呼ぶ。"""
        with self._lock:
            mag = self._mag
        st = self._mag_status_state()
        pose = self._map_pose_now()
        if not mag or not st or not st.get('alive') or not st.get('ok') or pose is None:
            return False
        if mag.get('stamp') and abs(time.time() - mag['stamp']) > 2.0:
            return False
        # heading_deg は磁北から時計まわり、yaw_deg は地図の x から反時計まわり。北は地図の yaw + heading の向き
        self._north_samples.append(_wrap_deg(pose['yaw_deg'] + mag['heading_deg'] + self.north_declination))
        self._north_set()
        return True

    def _north_set(self, adjust=None):
        a = self._north_samples
        with self._lock:
            old = self._north or {}
        adj = old.get('adjust', 0.0) if adjust is None else adjust
        measured, spread = old.get('measured'), old.get('spread')
        if a:
            mean = math.degrees(math.atan2(sum(math.sin(math.radians(v)) for v in a),
                                           sum(math.cos(math.radians(v)) for v in a)))
            dev = sorted(_wrap_deg(v - mean) for v in a)
            measured = _wrap_deg(mean + dev[len(dev) // 2])
            spread = (sum(d * d for d in dev) / len(dev)) ** 0.5 if len(dev) > 1 else None
        if measured is None:
            return None
        north = {'deg': round(_wrap_deg(measured + adj), 1), 'measured': round(measured, 1), 'adjust': adj,
                 'n': len(a) or old.get('n', 0), 'spread': round(spread, 1) if spread is not None else None}
        with self._lock:
            self._north = north
        return north

    def _exploring(self):
        r = self._route
        return r is not None and r['state'] == 'running' and r.get('kind') == 'explore'

    def north_adjust(self, deg):
        with self._lock:
            old = self._north
        if not old:
            return {'ok': False, 'error': 'まだ北が分かりません（地図作りの始めにはかります）'}
        north = self._north_set(max(-180.0, min(180.0, old.get('adjust', 0.0) + float(deg))))
        self._save_places()
        return {'ok': True, 'north': north}

    def request_mapping(self, req: MappingRequest):
        if req.action == 'cancel':
            if self._exploring():
                self._route_end('stopped', 'やめる指示')
            return {'ok': True}
        if req.action == 'north':
            return self.north_adjust(req.deg)
        if req.action != 'go':
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.nav_goal_pub is None:
            return {'ok': False, 'error': '地図の行き先が無効です'}
        r = self._route
        if r is not None and r['state'] == 'running':
            return {'ok': False, 'error': 'いま回っています。先にやめてください'}
        err = self._lidar_error()
        if err:
            return {'ok': False, 'error': err}
        auto = self._autonomy_state() or {}
        if not auto.get('alive') or not (auto.get('nav') or {}).get('available'):
            return {'ok': False, 'error': 'Nav2 が動いていません'}
        res = self.clear_map()
        if not res.get('ok'):
            return {'ok': False, 'error': res.get('reason') or '地図を消せない'}
        with self._lock:
            out0 = self._output
            self._route = {'state': 'running', 'kind': 'explore', 'phase': 'north', 'stops': [], 'names': [],
                           'index': 0, 'arrived': [], 'skipped': [], 'reason': '', 'retries': 0,
                           'stamp': round(time.time(), 1), 'map': '', 'id': int(time.time() * 1000),
                           '_sent': None, '_arrived_at': None, '_start': time.monotonic(), '_bad': [],
                           '_soft': [], '_again': False, '_seen': [], '_home': None, '_out0': out0}
        self.set_drive_output(min(out0 * 100.0, MAP_SPEED_PERCENT))
        self.get_logger().info('地図作り: 北をはかってから、まだ見ていない所を回る')
        return {'ok': True, 'route': self._route_state()}

    def _explore_next(self, now):
        """次の「はし」へ Nav2 で向かう。もうなければスタートにもどり、着いたら終わる。"""
        r = self._route
        with self._lock:
            grid = self._map_grid
        pose = self._map_pose_now()
        if grid is None or pose is None:
            if now - r['_start'] > MAP_NORTH_S + MAP_WAIT_S:
                self._route_end('failed', '地図か車の位置が来ない（SLAM）')
            return
        first = r['phase'] != 'explore'
        if r['_home'] is None:
            r['_home'] = {'name': 'スタート', 'x': round(pose['x'], 2), 'y': round(pose['y'], 2)}
        goal = None
        if len(r['stops']) < MAP_MAX_GOALS and now - r['_start'] < MAP_LIMIT_S - 60.0:
            marks = [tuple(m[:2]) for m in (self._autonomy_state() or {}).get('cliff_marks') or []]
            r['_cliffs'] = len(marks)
            # 着いた所に見えない所が残っても（物のかげ）、もうそこへは行かない
            goal = self._frontier(grid, pose, r['_bad'] + r['_soft'] + r['_seen'] + marks, marks)
            if goal is None and r['_soft'] and not r['_again']:
                # ほかに「はし」がない。とばした所（ハンガーラックの横のドア口など）へ、もう一度だけ行ってみる
                self.get_logger().info(f'地図作り: とばした「はし」{len(r["_soft"])} か所へ、もう一度行ってみる')
                r['_again'], r['_soft'] = True, []
                goal = self._frontier(grid, pose, r['_bad'] + r['_seen'] + marks, marks)
        if goal is None:
            home = r['_home']
            if first:
                self._route_end('failed', '行ける「はし」が見つからない（車のまわりがせまい？）')
                return
            if math.hypot(pose['x'] - home['x'], pose['y'] - home['y']) < 0.3:
                self._route_end('done', '')
                return
            if not r.get('_returning'):
                self.set_drive_output(MAP_HOME_SPEED_PERCENT)
            r['_returning'] = True
            stop = dict(home)
        else:
            stop = {'name': f'はし {len(r["stops"]) + 1}', 'x': goal['x'], 'y': goal['y']}
        r['stops'].append(stop)
        r['names'].append(stop['name'])
        r['index'] = len(r['stops']) - 1
        r['phase'], r['retries'], r['_dwell_at'] = 'explore', 0, None
        self._route_send(yaw=math.degrees(math.atan2(stop['y'] - pose['y'], stop['x'] - pose['x'])))
        self.get_logger().info(f'地図作り: {stop["name"]}へ x {stop["x"]} y {stop["y"]}')
        if first:
            res = self.request_drive_mode('auto')
            if not res.get('ok'):
                self._route_end('failed', res.get('error') or '自動にできない')

    def _frontier(self, grid, pose, bad, marks):
        """今の車の所から「はし」をえらぶ。車がせまい所（物の横）にいて道が出せないときは、スタートからえらぶ。

        スタートからえらぶときも、今の車のすぐ近く（さっき見えなかった所）はえらばない。
        """
        home = self._route['_home']
        here = (pose['x'], pose['y'])
        for start, near in ((here, []), ((home['x'], home['y']), [here])):
            goal = frontier_goal(grid, start, MAP_CLEAR_M, bad=bad + near, block=marks, block_m=MAP_CLIFF_M)
            if goal is not None:
                return goal
        return None

    # --- パトロール ---
    def _patrolling(self):
        r = self._route
        return r is not None and r['state'] == 'running' and r.get('kind') == 'patrol'

    def _patrol_stops(self):
        """(見る所の列, 地図, 車の位置, エラー)。緑の点と、地図を升に分けた「見る所」を、道のりが短くなる順に。"""
        with self._lock:
            grid = self._map_grid
            greens = [dict(p, green=True) for p in self._places]
        pose = self._map_pose_now()
        if grid is None:
            return None, None, None, 'まだ地図がありません'
        if pose is None:
            return None, None, None, '地図の上の車の位置がまだ分かりません'
        start = (pose['x'], pose['y'])
        spacing = PATROL_SPACING_M
        while True:
            pts = [q for q in coverage_points(grid, start, spacing, PATROL_CLEAR_M)
                   if all(math.hypot(q['x'] - g['x'], q['y'] - g['y']) >= PATROL_NEAR_GREEN_M for g in greens)]
            if len(pts) + len(greens) <= PATROL_MAX or spacing > 5.0:
                break
            spacing += 0.25
        stops = order_points(start, pts + greens[:PATROL_MAX])[:PATROL_MAX]
        n = 0
        for q in stops:
            if not q.get('green'):
                n += 1
                q['name'] = f'見る所 {n}'
        if not stops:
            return None, None, None, '地図に行ける所がありません'
        return stops, grid, pose, ''

    def patrol_plan(self):
        """回る所と順番だけ（走らない）。"""
        stops, _, pose, err = self._patrol_stops()
        if err:
            return {'ok': False, 'error': err}
        length, (px, py) = 0.0, (pose['x'], pose['y'])
        for q in stops:
            length += math.hypot(q['x'] - px, q['y'] - py)
            px, py = q['x'], q['y']
        return {'ok': True, 'stops': stops, 'line_m': round(length, 1)}

    def request_patrol(self, req: PatrolRequest):
        if req.action == 'cancel':
            if self._patrolling():
                self._route_end('stopped', 'やめる指示')
            return {'ok': True}
        if req.action != 'go':
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.nav_goal_pub is None:
            return {'ok': False, 'error': '地図の行き先が無効です'}
        r = self._route
        if r is not None and r['state'] == 'running':
            return {'ok': False, 'error': 'いま回っています。先にやめてください'}
        err = self._lidar_error()
        if err:
            return {'ok': False, 'error': err}
        auto = self._autonomy_state() or {}
        if not auto.get('alive') or not (auto.get('nav') or {}).get('available'):
            return {'ok': False, 'error': 'Nav2 が動いていません'}
        stops, grid, pose, err = self._patrol_stops()
        if err:
            return {'ok': False, 'error': err}
        with self._lock:
            home = dict(self._home) if self._home else None
        if home:
            stops.append(dict(home, name='ホーム', home=True))
        elif math.hypot(stops[-1]['x'] - pose['x'], stops[-1]['y'] - pose['y']) > 0.3:
            stops.append({'x': round(pose['x'], 2), 'y': round(pose['y'], 2), 'name': 'スタート', 'home': True})
        pid = time.strftime('%Y%m%d-%H%M%S')
        os.makedirs(os.path.join(self.patrol_dir, pid), exist_ok=True)
        with self._lock:
            loaded = self._map_loaded
            self._route = {'state': 'running', 'kind': 'patrol', 'pid': pid, 'stops': stops,
                           'names': [q['name'] for q in stops], 'index': 0, 'arrived': [], 'skipped': [],
                           'anomalies': [], 'reason': '', 'retries': 0, 'stamp': round(time.time(), 1),
                           'map': loaded.get('name') if loaded and loaded.get('ok') else '',
                           'id': int(time.time() * 1000), '_sent': None, '_arrived_at': None,
                           '_start': time.monotonic(), '_ref': grid, '_shots': [], '_objs': [],
                           '_cliff0': [tuple(m[:2]) for m in auto.get('cliff_marks') or []]}
        self.get_logger().info(f'パトロール: {len(stops)} か所 {" → ".join(q["name"] for q in stops)}')
        self._route_send()
        res = self.request_drive_mode('auto')
        if not res.get('ok'):
            self._route_end('failed', res.get('error') or '自動にできない')
            return {'ok': False, 'error': res.get('error') or '自動にできない'}
        return {'ok': True, 'route': self._route_state()}

    def _patrol_where(self, x, y):
        r = self._route
        cands = [q for q in r['stops'] if q.get('green')] + [q for q in r['stops'] if not q.get('green')]
        best = min(cands, key=lambda q: math.hypot(q['x'] - x, q['y'] - y))
        return f'{best["name"]}{" " if best["name"][-1:].isdigit() else ""}の近く'

    def _patrol_look(self):
        """止まって見る 1 回: 写真を残し、人（YOLO）と、地図になかった床の物（LiDAR）を見る。"""
        r = self._route
        pose = self._map_pose_now()
        frame, _ = self.latest_frame()
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            obstacle, ob_at, scan = self._obstacle, self._obstacle_stamp, self._scan
        shot = {'stop': r['stops'][r['index']]['name'], 'look': r['look'], 'photo': None,
                'x': pose['x'] if pose else None, 'y': pose['y'] if pose else None,
                'yaw_deg': pose['yaw_deg'] if pose else None}
        if frame:
            n = len(r['_shots'])
            try:
                with open(os.path.join(self.patrol_dir, r['pid'], f'{n}.jpg'), 'wb') as f:
                    f.write(frame)
                shot['photo'] = n
            except OSError as e:
                self.get_logger().warn(f'パトロールの写真を書けない: {e}')
        r['_shots'].append(shot)
        if pose is None:
            return
        yaw = math.radians(pose['yaw_deg'])
        if obstacle and now_ros - ob_at < 2.0:
            hfov = math.radians(float(obstacle.get('camera_hfov_deg') or 102.0))
            cam_x = float(obstacle.get('camera_offset_x_m') or 0.0)
            for d in obstacle.get('detections') or []:
                if d.get('label') not in PERSON_LABELS:
                    continue
                a = (0.5 - float(d.get('center_x', 0.5))) * hfov
                dist = d.get('distance')
                lx, ly = (cam_x + dist * math.cos(a), dist * math.sin(a)) if dist else (0.0, 0.0)
                x = pose['x'] + math.cos(yaw) * lx - math.sin(yaw) * ly
                y = pose['y'] + math.sin(yaw) * lx + math.cos(yaw) * ly
                if any(o['kind'] == 'person' and math.hypot(o['x'] - x, o['y'] - y) < 1.0 for o in r['anomalies']):
                    continue
                text = f'人がいる（{dist:.1f} m 先）' if dist else '人がいる'
                r['anomalies'].append({'kind': 'person', 'x': round(x, 2), 'y': round(y, 2), 'photo': shot['photo'],
                                       'where': self._patrol_where(x, y), 'text': text})
                self.get_logger().info(f'パトロール: {text} x {x:.2f} y {y:.2f}')
        if scan and now_ros - (scan.get('stamp') or 0) < 1.0:
            for o in new_objects(r['_ref'], pose, scan.get('points'), LIDAR_OFFSET):
                old = next((q for q in r['_objs'] if math.hypot(q['x'] - o['x'], q['y'] - o['y']) < 0.3), None)
                if old is None:
                    r['_objs'].append(dict(o, looks=1))
                else:
                    old['looks'] += 1

    def _patrol_photo_for(self, r, x, y):
        """(x, y) の方を向いて、いちばん近くでとった写真の番号。"""
        best, score = None, None
        for sh in r['_shots']:
            if sh['photo'] is None or sh['x'] is None:
                continue
            d = math.hypot(x - sh['x'], y - sh['y'])
            a = abs(_wrap_deg(math.degrees(math.atan2(y - sh['y'], x - sh['x'])) - sh['yaw_deg']))
            if d > 3.0 or a > 45.0:
                continue
            if score is None or d + a / 45.0 < score:
                best, score = sh['photo'], d + a / 45.0
        return best

    def _patrol_finish(self, r):
        """床の物（2 回以上見えた物）と、新しい段差の印を足して、結果を保存する。"""
        objs = sorted((o for o in r['_objs'] if o['looks'] >= 2), key=lambda o: -o['looks'])[:5]
        for o in objs:
            r['anomalies'].append({'kind': 'object', 'x': o['x'], 'y': o['y'],
                                   'photo': self._patrol_photo_for(r, o['x'], o['y']),
                                   'where': self._patrol_where(o['x'], o['y']), 'text': '床に物がある（地図になかった物）'})
        auto = self._autonomy_state() or {}
        for m in auto.get('cliff_marks') or []:
            x, y = m[0], m[1]
            if all(math.hypot(x - a, y - b) >= 0.15 for a, b in r['_cliff0']):
                r['anomalies'].append({'kind': 'cliff', 'x': x, 'y': y, 'photo': self._patrol_photo_for(r, x, y),
                                       'where': self._patrol_where(x, y), 'text': '新しい段差'})
        r['duration_s'] = round(time.monotonic() - r['_start'])
        report = dict({k: v for k, v in r.items() if not k.startswith('_')}, shots=r['_shots'])
        base = os.path.join(self.patrol_dir, r['pid'])
        try:
            os.makedirs(base, exist_ok=True)
            with open(os.path.join(base, 'report.json.tmp'), 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False)
            os.replace(os.path.join(base, 'report.json.tmp'), os.path.join(base, 'report.json'))
            for old in sorted(d for d in os.listdir(self.patrol_dir) if PATROL_ID_RE.fullmatch(d))[:-PATROL_KEEP]:
                for fn in os.listdir(os.path.join(self.patrol_dir, old)):
                    os.remove(os.path.join(self.patrol_dir, old, fn))
                os.rmdir(os.path.join(self.patrol_dir, old))
        except OSError as e:
            self.get_logger().warn(f'パトロールの結果を書けない: {e}')
        self.get_logger().info(f'パトロール: {r["state"]} 異常 {len(r["anomalies"])} とばした {len(r["skipped"])}')

    def patrol_reports(self):
        """保存したパトロールの結果（新しい順、写真の一覧なし）。"""
        out = []
        try:
            ids = sorted((d for d in os.listdir(self.patrol_dir) if PATROL_ID_RE.fullmatch(d)), reverse=True)
        except OSError:
            ids = []
        for pid in ids[:PATROL_KEEP]:
            try:
                with open(os.path.join(self.patrol_dir, pid, 'report.json'), encoding='utf-8') as f:
                    rep_ = json.load(f)
            except (OSError, ValueError):
                continue
            rep_.pop('shots', None)
            out.append(rep_)
        return {'reports': out}

    def patrol_photo(self, pid, n):
        if not PATROL_ID_RE.fullmatch(pid or ''):
            raise HTTPException(status_code=400, detail='id がおかしい')
        path = os.path.join(self.patrol_dir, pid, f'{int(n)}.jpg')
        if not os.path.exists(path):
            raise HTTPException(status_code=404, detail='ありません')
        return path

    def request_nav(self, req: NavRequest):
        """地図の行き先を autonomy_node へ送る。go のときは運転モードを自動にする。"""
        if req.action not in ('plan', 'go', 'cancel'):
            return {'ok': False, 'error': f'不明な指示: {req.action}'}
        if self.nav_goal_pub is None:
            return {'ok': False, 'error': '地図の行き先が無効です'}
        msg = String()
        msg.data = json.dumps({'action': req.action, 'x': req.x, 'y': req.y,
                               'yaw_deg': req.yaw_deg})
        self.nav_goal_pub.publish(msg)
        self.get_logger().info(f'行き先 {req.action}: x {req.x:.2f} y {req.y:.2f}')
        if req.action == 'go':
            return self.request_drive_mode('auto')
        return {'ok': True}

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
        status = host_status(req)
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._jetson_host = status
            self._jetson_host_stamp = now
            reboot = (self._jetson_reboot_at is not None
                      and now - self._jetson_reboot_at < JETSON_REBOOT_WINDOW)
            self._jetson_reboot_at = None
            poweroff = (self._jetson_poweroff_at is not None
                        and now - self._jetson_poweroff_at < JETSON_REBOOT_WINDOW)
            self._jetson_poweroff_at = None
            upgrade = (self._jetson_upgrade_at is not None
                       and now - self._jetson_upgrade_at < JETSON_REBOOT_WINDOW)
            self._jetson_upgrade_at = None
        if reboot:
            self.get_logger().warn('Jetson に再起動を伝えました')
        if poweroff:
            self.get_logger().warn('Jetson に電源を切ることを伝えました')
        if upgrade:
            self.get_logger().info('Jetson に更新を伝えました')
        return {'ok': True, 'reboot': reboot, 'poweroff': poweroff, 'upgrade': upgrade}

    def update_mypc_status(self, req: MyPCStatusRequest):
        """パソコンの状態を覚える（ダッシュボードの Jetson のカードで MyPC を選んだときに出す）。"""
        status = host_status(req)
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            self._mypc_host = status
            self._mypc_host_stamp = now
            asked = {k: now - t < JETSON_REBOOT_WINDOW for k, t in self._mypc_asked.items()}
            self._mypc_asked = {}
        for key, yes in asked.items():
            if yes:
                self.get_logger().warn(f'パソコンに {key} を伝えました')
        return {'ok': True, **{k: asked.get(k, False) for k in MYPC_ACTIONS}}

    def request_mypc(self, action: str):
        """次にパソコンから状態が届いたときの返事で、Windows の再起動・電源を切る、Ubuntu の更新を頼む。"""
        if action not in MYPC_ACTIONS:
            raise HTTPException(status_code=404, detail='reboot・poweroff・upgrade のどれかです')
        with self._lock:
            state = self._mypc_host_state()
            if not state or not state['alive']:
                raise HTTPException(status_code=409, detail='パソコンが未接続です')
            if not state.get(f'can_{action}'):
                raise HTTPException(status_code=503, detail=f'パソコンで {action} ができません')
            if action == 'upgrade' and state.get('upgrade_running'):
                raise HTTPException(status_code=409, detail='パソコンは更新中です')
            self._mypc_asked[action] = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn(f'ダッシュボードからパソコンの {action} を受け付けました')
        return {'ok': True}

    def _mypc_host_state(self):
        if self._mypc_host is None:
            return None
        age = self.get_clock().now().nanoseconds * 1e-9 - self._mypc_host_stamp
        return dict(self._mypc_host, alive=age < JETSON_STATUS_TIMEOUT, age_s=round(age, 1),
                    **{f'{k}_pending': k in self._mypc_asked for k in MYPC_ACTIONS})

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

    def _stackchan_bases(self):
        """stackchan_bridge の URL。stackchan_bridge_url か、ミニコンとパソコン（控え）が送ってくる IP アドレス
        （Tailscale を先に）。ミニコンが未接続ならパソコンを先に。"""
        if self.stackchan_bridge_url:
            return [self.stackchan_bridge_url]
        with self._lock:
            hosts = [self._jetson_host_state(), self._mypc_host_state()]
        if not (hosts[0] or {}).get('alive'):
            hosts.reverse()
        bases = []
        for state in hosts:
            ips = [ip for ip in (state or {}).get('ips') or [] if ip.get('addr')]
            ips.sort(key=lambda ip: ip.get('iface') != 'tailscale0')
            if ips:
                bases.append(f"http://{ips[0]['addr']}:{STACKCHAN_BRIDGE_PORT}")
        return bases

    def stackchan_request(self, path, method='GET', timeout=STACKCHAN_TIMEOUT):
        """stackchan_bridge に聞いて、(中身, Content-Type) を返す（スタックちゃんのカード用、表示だけ）。
        ミニコンにつながらなければ、パソコンの控えに聞く。"""
        bases = self._stackchan_bases()
        if not bases:
            raise HTTPException(status_code=503, detail='ミニコンの IP アドレスがまだ届いていません')
        for base in bases[:-1]:
            try:
                return self._stackchan_get(base, path, method, timeout)
            except HTTPException as exc:
                if exc.status_code != 503:
                    raise
        return self._stackchan_get(bases[-1], path, method, timeout)

    def _stackchan_get(self, base, path, method, timeout):
        req = urllib.request.Request(base + path, data=b'' if method == 'POST' else None, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as res:
                return res.read(), res.headers.get('Content-Type')
        except urllib.error.HTTPError as exc:
            try:
                detail = json.loads(exc.read()).get('detail')
            except (ValueError, AttributeError):
                detail = None
            raise HTTPException(status_code=exc.code if exc.code in (400, 404, 409) else 502,
                                detail=detail or f'Jetson の stackchan_bridge が {exc.code} を返しました')
        except (urllib.error.URLError, OSError):
            raise HTTPException(status_code=503, detail='Jetson の stackchan_bridge につながりません')

    def stackchan_json(self, path, method='GET', timeout=STACKCHAN_TIMEOUT):
        body, _ = self.stackchan_request(path, method, timeout)
        try:
            return json.loads(body)
        except ValueError:
            raise HTTPException(status_code=502, detail='Jetson の stackchan_bridge の返事を読めません')

    def request_jetson_poweroff(self):
        """次に Jetson から状態が届いたときの返事で、電源を切ることを頼む。"""
        with self._lock:
            state = self._jetson_host_state()
            if not state or not state['alive']:
                raise HTTPException(status_code=409, detail='Jetson が未接続です')
            if not state.get('can_poweroff'):
                raise HTTPException(
                    status_code=503,
                    detail='Jetson の jetson-reboot.sudoers に poweroff が入っていません')
            self._jetson_poweroff_at = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().warn('ダッシュボードから Jetson の電源を切ることを受け付けました')
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
        return self._shutdown_pi(REBOOT_CMD, '再起動')

    def poweroff_pi(self):
        """止まっているときだけ、ラズパイの電源を切る（10 秒後。返事を先に返す）。"""
        return self._shutdown_pi(POWEROFF_CMD, '電源オフ')

    def _shutdown_pi(self, cmd, label):
        if self._moving():
            raise HTTPException(status_code=409, detail=f'走行中は{label}できません（先に止めてください）')
        if not _can_sudo(cmd):
            raise HTTPException(status_code=503,
                                detail=f'ラズパイの ai-car-reboot.sudoers に {cmd[-1]} が入っていません')
        self.stop()
        self.get_logger().warn(f'ダッシュボードからラズパイの{label}を受け付けました（{PI_REBOOT_DELAY:.0f} 秒後）')
        threading.Thread(target=self._reboot_pi_later, args=(cmd,), daemon=True).start()
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

    def _sync_diary(self):
        try:
            added = sync_dev_diary(self.repo_dir, self.diary_db)
            self.get_logger().info(f'開発日記の DB を更新しました（{added} 件追加）')
        except Exception as e:  # noqa: BLE001  起動を止めない
            self.get_logger().warning(f'開発日記の DB を更新できません: {e}')

    def _reboot_pi_later(self, cmd):
        """10 秒待ち（そのあいだも Jetson の再起動・電源を切る頼みを受け付ける）、頼みが残っていれば伝え終わるまで待ってから cmd を実行する。"""
        time.sleep(PI_REBOOT_DELAY)
        deadline = time.monotonic() + JETSON_REBOOT_WINDOW
        waited = False
        while time.monotonic() < deadline:
            with self._lock:
                pending = self._jetson_reboot_at is not None or self._jetson_poweroff_at is not None
            if not pending:
                break
            waited = True
            time.sleep(0.2)
        if waited:
            time.sleep(1.0)
        subprocess.run(cmd, timeout=30, check=False)

    def _jetson_host_state(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._jetson_host is None:
            return None
        age = now - self._jetson_host_stamp
        state = dict(self._jetson_host)
        state['alive'] = age < JETSON_STATUS_TIMEOUT
        state['age_s'] = round(age, 1)
        state['reboot_pending'] = self._jetson_reboot_at is not None
        state['poweroff_pending'] = self._jetson_poweroff_at is not None
        state['upgrade_pending'] = self._jetson_upgrade_at is not None
        return state

    @staticmethod
    def _clamp_output(value: float) -> float:
        return max(0.1, min(1.0, value))

    def set_drive_output(self, percent: float):
        """「出力」を変え、drive_mode_node（自動の上限）へも送る。"""
        with self._lock:
            self._output = self._clamp_output(percent / 100.0)
            output = self._output
        self.output_pub.publish(Float32(data=output))
        return {'output_percent': round(output * 100)}

    def publish_cmd_vel(self, linear_x: float, linear_y: float, angular_z: float):
        """正規化済み (-1.0〜1.0) の指令値を「出力」と最大速度にスケールして publish する。"""
        twist = Twist()
        with self._lock:
            output = self._output
        twist.linear.x = self._clamp(linear_x) * output * self.max_linear
        twist.linear.y = self._clamp(linear_y) * output * self.max_linear
        twist.angular.z = self._clamp(angular_z) * output * self.max_angular
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
                'lidar': self._lidar_health_locked(),
                'imu': self._imu,
                'mag': self._mag,
                'mag_status': self._mag_status_state(),
                'odom': self._odom,
                'system': self._system,
                'obstacle': self._obstacle,
                'camera': self._camera_state(),
                'drive_mode': self._drive_mode_state(),
                'autonomy': self._autonomy_state(),
                'cliff': self._cliff_state(),
                'slam': self._slam_state(),
                'route': self._route_state(),
                'jetson_host': self._jetson_host_state(),
                'mypc_host': self._mypc_host_state(),
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
                    'output_percent': round(self._output * 100),
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
        data = load_architecture(self.architecture_config, params, self.repo_dir)
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
        if isinstance(data.get('overview'), dict):
            data['overview']['version'] = version()
        return data

    @app.get('/api/version')
    def version():
        return load_version(get_package_share_directory('ai_car_web'), node.repo_dir)

    @app.get('/api/program')
    def program(file: str):
        data = read_program(node.architecture_config, node.repo_dir, file)
        if data is None:
            raise HTTPException(status_code=404, detail='一覧にないファイルか、読めません')
        return data

    @app.get('/api/dev_diary')
    def dev_diary(days: int = 7, before: str = '', tag: str = ''):
        return query_dev_diary(node.repo_dir, node.diary_db, days=min(max(days, 1), 365),
                               before=before, tag=tag)

    @app.post('/api/cmd_vel', dependencies=[Depends(require_token)])
    def cmd_vel(req: CmdVelRequest):
        return node.publish_cmd_vel(req.linear_x, req.linear_y, req.angular_z)

    @app.post('/api/drive_output', dependencies=[Depends(require_token)])
    def drive_output(req: DriveOutputRequest):
        return node.set_drive_output(req.percent)

    @app.post('/api/stop', dependencies=[Depends(require_token)])
    def stop():
        return node.stop()

    @app.post('/api/jetson/detections', dependencies=[Depends(require_token)])
    def jetson_detections(req: JetsonDetectionsRequest):
        return node.publish_jetson_detections(req)

    @app.post('/api/jetson/status', dependencies=[Depends(require_token)])
    def jetson_status(req: JetsonStatusRequest):
        return node.update_jetson_status(req)

    @app.post('/api/mypc/status', dependencies=[Depends(require_token)])
    def mypc_status(req: MyPCStatusRequest):
        return node.update_mypc_status(req)

    @app.post('/api/mypc/{action}', dependencies=[Depends(require_token)])
    def mypc_action(action: str):
        return node.request_mypc(action)

    @app.post('/api/jetson/reboot', dependencies=[Depends(require_token)])
    def jetson_reboot():
        return node.request_jetson_reboot()

    @app.post('/api/jetson/poweroff', dependencies=[Depends(require_token)])
    def jetson_poweroff():
        return node.request_jetson_poweroff()

    @app.post('/api/jetson/upgrade', dependencies=[Depends(require_token)])
    def jetson_upgrade():
        return node.request_jetson_upgrade()

    @app.post('/api/system/reboot', dependencies=[Depends(require_token)])
    def system_reboot():
        return node.reboot_pi()

    @app.post('/api/system/poweroff', dependencies=[Depends(require_token)])
    def system_poweroff():
        return node.poweroff_pi()

    @app.post('/api/system/upgrade', dependencies=[Depends(require_token)])
    def system_upgrade():
        return node.upgrade_pi()

    @app.get('/api/drive_mode')
    def drive_mode():
        return node.telemetry()['drive_mode'] or {'mode': None, 'alive': False}

    @app.post('/api/drive_mode', dependencies=[Depends(require_token)])
    def set_drive_mode(req: DriveModeRequest):
        return node.request_drive_mode(req.mode)

    @app.post('/api/nav', dependencies=[Depends(require_token)])
    def nav(req: NavRequest):
        return node.request_nav(req)

    @app.get('/api/places')
    def places():
        return node.places()

    @app.post('/api/places', dependencies=[Depends(require_token)])
    def edit_place(req: PlaceRequest):
        return node.edit_place(req)

    @app.post('/api/route', dependencies=[Depends(require_token)])
    def route(req: RouteRequest):
        return node.request_route(req)

    @app.post('/api/home', dependencies=[Depends(require_token)])
    def home(req: HomeRequest):
        return node.request_home(req)

    @app.get('/api/patrol')
    def patrol_reports():
        return node.patrol_reports()

    @app.get('/api/patrol/plan')
    def patrol_plan():
        return node.patrol_plan()

    @app.get('/api/patrol/{pid}/{n}.jpg')
    def patrol_photo(pid: str, n: int):
        return FileResponse(node.patrol_photo(pid, n), media_type='image/jpeg')

    @app.post('/api/patrol', dependencies=[Depends(require_token)])
    def patrol(req: PatrolRequest):
        return node.request_patrol(req)

    @app.post('/api/cliff_marks', dependencies=[Depends(require_token)])
    def cliff_marks(req: CliffMarkRequest):
        return node.edit_cliff_mark(req)

    @app.get('/api/slam/map.png')
    def slam_map():
        png = node.map_png()
        if png is None:
            return Response(status_code=503)
        return Response(content=png, media_type='image/png', headers={'Cache-Control': 'no-store'})

    @app.post('/api/slam/save', dependencies=[Depends(require_token)])
    def slam_save(req: SlamSaveRequest | None = None):
        return node.save_map(req.name if req else '')

    @app.get('/api/slam/maps')
    def slam_maps():
        return node.list_maps()

    @app.get('/api/slam/maps/{map_id}/map.png')
    def slam_map_png(map_id: str):
        return FileResponse(node.map_file(map_id, '.png'), media_type='image/png')

    @app.get('/api/slam/maps/{map_id}/photo.jpg')
    def slam_map_photo(map_id: str):
        return FileResponse(node.map_file(map_id, '.jpg'), media_type='image/jpeg')

    @app.post('/api/slam/maps/load', dependencies=[Depends(require_token)])
    def slam_map_load(req: SlamMapRequest):
        return node.load_map(req.id)

    @app.post('/api/slam/maps/rename', dependencies=[Depends(require_token)])
    def slam_map_rename(req: SlamMapRequest):
        return node.rename_map(req.id, req.name)

    @app.post('/api/slam/maps/delete', dependencies=[Depends(require_token)])
    def slam_map_delete(req: SlamMapRequest):
        return node.delete_map(req.id)

    @app.post('/api/slam/clear', dependencies=[Depends(require_token)])
    def slam_clear():
        return node.clear_map()

    @app.post('/api/slam/mapping', dependencies=[Depends(require_token)])
    def slam_mapping(req: MappingRequest):
        return node.request_mapping(req)

    @app.post('/api/imu/mag_calibrate', dependencies=[Depends(require_token)])
    def imu_mag_calibrate():
        return node.start_mag_calibration()

    @app.get('/api/stackchan/status', dependencies=[Depends(require_token)])
    def stackchan_status():
        return node.stackchan_json('/status')

    @app.post('/api/stackchan/photo', dependencies=[Depends(require_token)])
    def stackchan_photo():
        return node.stackchan_json('/photo', 'POST', STACKCHAN_PHOTO_TIMEOUT)

    @app.get('/api/stackchan/photo.jpg', dependencies=[Depends(require_token)])
    def stackchan_photo_jpg():
        body, ctype = node.stackchan_request('/photo.jpg')
        return Response(content=body, media_type=ctype or 'image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.post('/api/stackchan/enroll', dependencies=[Depends(require_token)])
    def stackchan_enroll(name: str):
        return node.stackchan_json('/enroll?name=' + urllib.parse.quote(name.strip()), 'POST',
                                   STACKCHAN_PHOTO_TIMEOUT)

    @app.post('/api/stackchan/backend', dependencies=[Depends(require_token)])
    def stackchan_backend(to: str):
        if to not in ('jetson', 'pc'):
            raise HTTPException(status_code=400, detail='to は jetson か pc')
        return node.stackchan_json(f'/backend?to={to}', 'POST')

    @app.post('/api/stackchan/mode', dependencies=[Depends(require_token)])
    def stackchan_mode(to: str):
        if to not in ('local', 'net'):
            raise HTTPException(status_code=400, detail='to は local か net')
        return node.stackchan_json(f'/mode?to={to}', 'POST', STACKCHAN_MODE_TIMEOUT)

    @app.post('/api/stackchan/pc_voice', dependencies=[Depends(require_token)])
    def stackchan_pc_voice(speaker: int):
        return node.stackchan_json(f'/pc_voice?speaker={speaker}', 'POST')

    @app.post('/api/stackchan/stream', dependencies=[Depends(require_token)])
    def stackchan_stream(on: int = 0):
        return node.stackchan_json(f'/camera/stream?on={1 if on else 0}', 'POST')

    @app.get('/api/stackchan/frame.jpg', dependencies=[Depends(require_token)])
    def stackchan_frame_jpg():
        body, ctype = node.stackchan_request('/camera/frame.jpg')
        return Response(content=body, media_type=ctype or 'image/jpeg', headers={'Cache-Control': 'no-store'})

    @app.get('/api/camera/snapshot')
    def snapshot():
        frame, _ = node.latest_frame()
        if frame is None:
            return Response(status_code=503)
        return Response(content=frame, media_type='image/jpeg')

    def mjpeg(latest):
        interval = 1.0 / max(node.camera_stream_rate, 0.1)

        # 配信周期と同じ間隔で寝ると位相ずれで新フレームを取り逃がし実効レートが
        # 半減するため、短い周期で監視して配信間隔だけを守る。
        poll = min(interval / 4.0, 0.005)

        async def frames():
            last = -1
            next_at = 0.0
            while True:
                frame, count = latest()
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

    @app.get('/api/camera/stream')
    def stream():
        return mjpeg(node.latest_frame)

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
    if node.repo_dir:
        video_dir = os.path.join(node.repo_dir, VIDEO_DIR)
        os.makedirs(video_dir, exist_ok=True)
        app.mount('/videos', StaticFiles(directory=video_dir), name='videos')
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
