from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[],
            arguments=[
                '-configuration_directory', os.path.join(
                    get_package_share_directory('mybot_cartographer'), 'config'),
                '-configuration_basename', 'mybot.lua'
            ],
            remappings=[('/scan', '/scan')],
        ),
    ])
