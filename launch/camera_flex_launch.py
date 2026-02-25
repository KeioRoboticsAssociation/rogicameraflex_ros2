import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('rogicameraflex_ros2'),
        'config', 'config.yaml')

    return LaunchDescription([
        Node(
            package='rogicameraflex_ros2',
            executable='camera_flex_node',
            name='rogicameraflex_ros2',
            parameters=[{'config_file': config_path}],
            output='screen',
        ),
    ])
