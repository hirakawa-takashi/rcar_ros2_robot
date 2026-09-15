import os
import platform

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _rpicam_env():
    """Raspberry Pi 版 libcamera を優先させる環境変数を返す。

    Ubuntu 24.04 の libcamera は raspi カーネルのメディアエンティティ名と
    互換性がなくカメラを列挙できないため、~/opt/rpicam に導入した
    Raspberry Pi 版 libcamera があればそちらを先に読み込む。
    """
    libdir = os.path.join(
        os.path.expanduser('~'), 'opt', 'rpicam', 'lib',
        f'{platform.machine()}-linux-gnu')
    if not os.path.isdir(libdir):
        return {}
    return {'LD_LIBRARY_PATH': os.pathsep.join(
        [libdir, os.environ.get('LD_LIBRARY_PATH', '')]).rstrip(os.pathsep)}


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ai_car_web'), 'config', 'dashboard.yaml')

    params_file = LaunchConfiguration('params_file')
    use_system_monitor = LaunchConfiguration('use_system_monitor')
    use_camera = LaunchConfiguration('use_camera')
    use_lidar = LaunchConfiguration('use_lidar')
    use_perception = LaunchConfiguration('use_perception')
    use_joy = LaunchConfiguration('use_joy')
    use_seg_display = LaunchConfiguration('use_seg_display')
    use_imu = LaunchConfiguration('use_imu')
    use_drive_mode = LaunchConfiguration('use_drive_mode')
    use_autonomy = LaunchConfiguration('use_autonomy')

    try:
        get_package_share_directory('camera_ros')
        camera_available = True
    except PackageNotFoundError:
        camera_available = False

    camera_nodes = [
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            parameters=[params_file],
            additional_env=_rpicam_env(),
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(use_camera),
        ),
    ] if camera_available else []

    try:
        get_package_share_directory('rplidar_ros')
        lidar_available = True
    except PackageNotFoundError:
        lidar_available = False

    lidar_nodes = [
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(use_lidar),
        ),
    ] if lidar_available else []

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ダッシュボードのパラメータファイル'),
        DeclareLaunchArgument('use_system_monitor', default_value='true',
                              description='システム監視ノードを起動する'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='camera_ros のカメラノードを起動する'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='rplidar_ros の LiDAR ノードを起動する'),
        DeclareLaunchArgument('use_perception', default_value='true',
                              description='障害物判定ノード（LiDAR 主 + AI HAT+）を起動する'),
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='ゲームパッド（F710）手動操作ノードを起動する'),
        DeclareLaunchArgument('use_seg_display', default_value='true',
                              description='TM1637 7セグ表示ノードを起動する'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='BNO055 IMU ノードを起動する'),
        DeclareLaunchArgument('use_drive_mode', default_value='true',
                              description='運転モード管理（手動/自動/停止 → /cmd_vel）を起動する'),
        DeclareLaunchArgument('use_autonomy', default_value='true',
                              description='自律走行ノード（LiDAR 反応型）を起動する'),
        Node(
            package='ai_car_web',
            executable='dashboard_node',
            name='dashboard_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='ai_car_web',
            executable='system_monitor_node',
            name='system_monitor_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_system_monitor),
        ),
        Node(
            package='ai_car_web',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_perception),
        ),
        Node(
            package='ai_car_web',
            executable='joy_teleop_node',
            name='joy_teleop_node',
            output='screen',
            parameters=[params_file],
            condition=IfCondition(use_joy),
        ),
        Node(
            package='ai_car_web',
            executable='seg_display_node',
            name='seg_display_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_seg_display),
        ),
        Node(
            package='ai_car_web',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_imu),
        ),
        Node(
            package='ai_car_web',
            executable='drive_mode_node',
            name='drive_mode_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_drive_mode),
        ),
        Node(
            package='ai_car_web',
            executable='autonomy_node',
            name='autonomy_node',
            output='screen',
            parameters=[params_file],
            respawn=True,
            respawn_delay=2.0,
            condition=IfCondition(use_autonomy),
        ),
        *camera_nodes,
        *lidar_nodes,
    ])
