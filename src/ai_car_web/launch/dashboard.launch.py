import os

from ament_index_python.packages import get_package_share_directory
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

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='ダッシュボードのパラメータファイル'),
        DeclareLaunchArgument('use_system_monitor', default_value='true',
                              description='システム監視ノードを起動する'),
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
    ])
