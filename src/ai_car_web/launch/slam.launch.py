"""slam_toolbox で地図を作り、地図の上の自己位置を出す。

robot_state_publisher で base_footprint → base_link → laser の TF を出し、
odom_source:=none（既定）のときは odom → base_footprint を固定の TF にする。
dashboard.launch.py の use_slam:=true からも読み込む。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    slam_params = LaunchConfiguration('slam_params_file')
    use_rsp = LaunchConfiguration('use_robot_state_publisher')
    xacro_file = os.path.join(
        get_package_share_directory('ai_car_description'), 'urdf', 'ai_car.xacro')
    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(
                get_package_share_directory('ai_car_web'), 'config', 'slam_toolbox.yaml'),
            description='slam_toolbox のパラメータファイル'),
        DeclareLaunchArgument(
            'odom_source', default_value='none',
            description='none: odom → base_footprint を固定（スキャンの照合だけで位置を出す）。'
                        'topic: 別のノード（車輪のオドメトリ）がこの TF を出す'),
        DeclareLaunchArgument(
            'use_robot_state_publisher', default_value='true',
            description='URDF から base_link → laser などの TF を出す'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            }],
            condition=IfCondition(use_rsp),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_footprint',
            output='screen',
            arguments=['--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
            condition=LaunchConfigurationEquals('odom_source', 'none'),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'slam_params_file': slam_params,
            }.items(),
        ),
    ])
