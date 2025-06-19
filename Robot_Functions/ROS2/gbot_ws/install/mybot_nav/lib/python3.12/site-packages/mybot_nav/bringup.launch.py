from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    carto_dir = get_package_share_directory('mybot_cartographer')
    nav_dir = get_package_share_directory('mybot_nav')

    return LaunchDescription([
        # Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')],
            arguments=[
                '-configuration_directory', os.path.join(carto_dir, 'config'),
                '-configuration_basename', 'mybot.lua'
            ],
            remappings=[('/scan', '/scan')],
        ),

        # Map publisher
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen'
        ),

        # Nav2 core
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),
        Node(
            package='nav2_behavior_tree',
            executable='bt_navigator',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'planner_server',
                    'bt_navigator'
                ]
            }]
        )
    ])
