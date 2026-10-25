"""一条命令拉起整条链路：ros2 launch flight_ui flight.launch.py

包含：MicroXRCEAgent(DDS 桥) + PX4/Gazebo + 摇杆守护 + 遥测缓存 + Web 后端。

常用参数：
  ros2 launch flight_ui flight.launch.py                # 默认
  ros2 launch flight_ui flight.launch.py headless:=false  # 带 Gazebo 图形窗口
  ros2 launch flight_ui flight.launch.py remote:=false    # 不要 Web 后端（纯终端飞）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project = '/home/wiki_wu/flight_project'
    ws = project

    headless = LaunchConfiguration('headless')
    remote = LaunchConfiguration('remote')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='true',
                              description='true=不开 Gazebo 图形窗口（省 GPU）'),
        DeclareLaunchArgument('remote', default_value='true',
                              description='true=启动 Web 遥控器后端（:8000）'),
        DeclareLaunchArgument('port', default_value='8000',
                              description='Web 后端端口'),

        LogInfo(msg='启动 MicroXRCEAgent ...'),
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='log',
            name='micro_xrce_agent',
        ),

        LogInfo(msg='启动 PX4 + Gazebo ...'),
        ExecuteProcess(
            cmd=['bash', '-lc',
                 f'cd {project} && HEADLESS=${{HEADLESS:-1}} '
                 f'./scripts/run_sitl_combined.sh'],
            additional_env={'HEADLESS': '1'},
            output='log',
            name='px4_sitl',
        ),

        LogInfo(msg='启动摇杆守护(50 Hz) ...'),
        ExecuteProcess(
            cmd=['python3', f'{project}/scripts/stick_keeper.py'],
            output='log',
            name='stick_keeper',
        ),

        LogInfo(msg='启动 Web 后端 ...', condition=IfCondition(remote)),
        ExecuteProcess(
            cmd=[f'{ws}/install/flight_ui_cpp/lib/flight_ui_cpp/rc_server'],
            output='log',
            name='rc_server',
            condition=IfCondition(remote),
        ),
    ])
