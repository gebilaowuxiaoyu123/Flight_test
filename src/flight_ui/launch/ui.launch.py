from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='flight_ui',
            executable='ui_server',
            name='flight_ui',
            output='screen',
            arguments=['--host', '0.0.0.0', '--port', '8000'],
        ),
    ])
