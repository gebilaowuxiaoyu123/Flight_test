"""只起 Web 遥控器后端（仿真已经在跑时用）：ros2 launch flight_ui remote.launch.py

默认参数来自 flight_ui_cpp/config/flight_ui.yaml，命令行用 port:= 可覆盖。
浏览器打开 http://127.0.0.1:8000/
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('flight_ui_cpp'), 'config', 'flight_ui.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000',
                              description='HTTP/WebSocket 端口，覆盖 yaml 里的值'),
        Node(
            package='flight_ui_cpp',
            executable='rc_server',
            name='flight_ui',                 # 必须和 yaml 里的顶层键一致
            output='screen',
            parameters=[params_file, {'port': LaunchConfiguration('port')}],
        ),
    ])
