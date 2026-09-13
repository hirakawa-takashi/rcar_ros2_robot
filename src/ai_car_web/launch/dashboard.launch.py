import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ai_car_web'), 'config', 'dashboard.yaml')

    params_file = LaunchConfiguration('params_file')
    use_system_monitor = LaunchConfiguration('use_system_monitor')
    use_camera = LaunchConfiguration('use_camera')

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
            condition=IfCondition(use_camera),
        ),
    ] if camera_available else []

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ダッシュボードのパラメータファイル'),
        DeclareLaunchArgument('use_system_monitor', default_value='true',
                              description='システム監視ノードを起動する'),
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='camera_ros のカメラノードを起動する'),
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
        *camera_nodes,
    ])
