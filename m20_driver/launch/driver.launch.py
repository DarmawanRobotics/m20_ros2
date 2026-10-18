#!/usr/bin/env python3
"""The M20 driver alone, in the robot namespace (m20_bringup starts everything)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare namespace and params_file, start m20_driver."""
    default_params = os.path.join(
        get_package_share_directory('m20_driver'), 'config', 'driver_params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='m20'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='m20_driver',
            executable='driver_node',
            name='m20_driver',
            namespace=LaunchConfiguration('namespace'),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ])
