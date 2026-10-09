"""Unified nav_bridge launch; robot_type is selected in config/nav_bridge.yaml."""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_node(context):
    pkg_dir = get_package_share_directory('nav_bridge')
    selector_file = os.path.join(pkg_dir, 'config', 'nav_bridge.yaml')
    with open(selector_file, encoding='utf-8') as stream:
        default_type = yaml.safe_load(stream).get('robot_type', 'x30')
    robot_type = LaunchConfiguration('robot_type').perform(context) or default_type
    if robot_type == 'x30':
        executable = 'x30_nav_bridge_node'
        params_file = os.path.join(pkg_dir, 'config', 'x30_params.yaml')
    elif robot_type == 'd1_max':
        executable = 'd1_max_nav_bridge_node'
        params_file = os.path.join(pkg_dir, 'config', 'd1_max_params.yaml')
    elif robot_type == 'cyvet':
        executable = 'cyvet_nav_bridge_node'
        params_file = os.path.join(pkg_dir, 'config', 'cyvet_params.yaml')
    else:
        raise RuntimeError(f'Unsupported robot_type: {robot_type}')

    params_override = LaunchConfiguration('params_file').perform(context)
    if params_override:
        params_file = params_override
    environment = {}
    if robot_type == 'cyvet':
        cyclone_file = LaunchConfiguration('cyclonedds_config').perform(context)
        environment = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
        if cyclone_file:
            environment['CYCLONEDDS_URI'] = 'file://' + os.path.abspath(cyclone_file)
        elif not os.environ.get('CYCLONEDDS_URI'):
            cyclone_file = os.path.join(pkg_dir, 'config', 'cyvet_cyclonedds.xml')
            environment['CYCLONEDDS_URI'] = 'file://' + os.path.abspath(cyclone_file)
    nav_bridge_node = Node(
        package='nav_bridge',
        executable=executable,
        name='nav_bridge_node',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
        additional_env=environment,
    )

    return [nav_bridge_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value=''),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('cyclonedds_config', default_value=''),
        OpaqueFunction(function=launch_node),
    ])
