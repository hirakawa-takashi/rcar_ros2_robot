"""Nav2（Level 2: 地図で行き先へ行く）を起動する。

slam_toolbox の地図（/map）と自己位置（map → odom の TF）を使い、
controller_server・planner_server・behavior_server・bt_navigator を lifecycle_manager で active にする。
速度指令（cmd_vel）は /cmd_vel_nav に出し、autonomy_node が AUTO で行き先があるときだけ /cmd_vel_auto へ中継する。
dashboard.launch.py の use_nav2:=true からも読み込む（slam_toolbox が active になってから起動するよう遅らせる）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator')
PACKAGES = {
    'controller_server': 'nav2_controller',
    'planner_server': 'nav2_planner',
    'behavior_server': 'nav2_behaviors',
    'bt_navigator': 'nav2_bt_navigator',
}


def generate_launch_description():
    params = LaunchConfiguration('nav2_params_file')
    cmd_topic = LaunchConfiguration('nav_cmd_topic')
    nodes = [
        Node(
            package=PACKAGES[name],
            executable=name,
            name=name,
            output='screen',
            parameters=[params],
            remappings=[('cmd_vel', cmd_topic)],
        )
        for name in NAV_NODES
    ]
    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                get_package_share_directory('ai_car_web'), 'config', 'nav2.yaml'),
            description='Nav2 のパラメータファイル'),
        DeclareLaunchArgument(
            'nav_cmd_topic', default_value='/cmd_vel_nav',
            description='Nav2 の速度指令の出先（autonomy_node が AUTO のときだけ中継する）'),
        *nodes,
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params],
        ),
    ])
