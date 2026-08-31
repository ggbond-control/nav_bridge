"""Unified nav_bridge launch; robot_type is selected in config/nav_bridge.yaml."""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_bridge')
    selector_file = os.path.join(pkg_dir, 'config', 'nav_bridge.yaml')
    with open(selector_file, encoding='utf-8') as stream:
        robot_type = yaml.safe_load(stream).get('robot_type', 'x30')
    if robot_type == 'x30':
        executable = 'x30_nav_bridge_node'
        params_file = os.path.join(pkg_dir, 'config', 'x30_params.yaml')
    elif robot_type == 'd1_max':
        executable = 'd1_max_nav_bridge_node'
        params_file = os.path.join(pkg_dir, 'config', 'd1_max_params.yaml')
    else:
        raise RuntimeError(f'Unsupported robot_type: {robot_type}')

    nav_bridge_node = Node(
        package='nav_bridge',
        executable=executable,
        name='nav_bridge_node',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        nav_bridge_node,
    ])
