"""只起 Web 遥控器后端（仿真已经在跑时用）：ros2 launch flight_ui remote.launch.py

浏览器打开 http://127.0.0.1:8000/
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    binary = '/home/wiki_wu/flight_project/install/flight_ui_cpp/lib/flight_ui_cpp/rc_server'

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8000'),
        ExecuteProcess(cmd=[binary], output='screen', name='rc_server'),
    ])
