from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    csv_path = LaunchConfiguration('csv_path')
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/depth_camera'),
        DeclareLaunchArgument('csv_path', default_value='/tmp/optical_flow_cpp.csv'),
        Node(
            package='flight_perception_cpp',
            executable='min_depth',
            name='min_depth',
            output='screen',
            parameters=[{'image_topic': image_topic}],
        ),
        Node(
            package='flight_perception_cpp',
            executable='optical_flow_logger',
            name='optical_flow_logger',
            output='screen',
            parameters=[{'csv_path': csv_path}],
        ),
    ])
