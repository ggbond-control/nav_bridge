"""Nav Bridge launch file for X30 robot dog."""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('nav_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'x30_params.yaml')

    nav_bridge_node = Node(
        package='nav_bridge',
        executable='nav_bridge_node',
        name='nav_bridge_node',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        nav_bridge_node,
    ])
