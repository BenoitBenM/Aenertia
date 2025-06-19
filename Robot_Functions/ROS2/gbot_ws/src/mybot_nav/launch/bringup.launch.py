from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    carto_dir = get_package_share_directory('mybot_cartographer')
    nav_dir   = get_package_share_directory('mybot_nav')

    return LaunchDescription([
        # Cartographer SLAM Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[],
            arguments=[
                '-configuration_directory', os.path.join(carto_dir, 'config'),
                '-configuration_basename', 'mybot.lua'
            ],
            remappings=[('/scan', '/scan')],
        ),

        # Map Publisher Node (to provide /map topic)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen'
        ),

        # Nav2 Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),

        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),

        # Nav2 Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[os.path.join(nav_dir, 'config', 'nav2_params.yaml')]
        ),

        # Nav2 Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
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
