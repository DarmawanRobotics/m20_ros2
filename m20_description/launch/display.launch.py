#!/usr/bin/env python3
"""The M20 URDF in RViz, with sliders for the joints (no robot needed)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the description, a joint state publisher and RViz."""
    share = get_package_share_directory('m20_description')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='m20'),
        DeclareLaunchArgument('gui', default_value='true', description='joint sliders'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'description.launch.py')),
            launch_arguments={'namespace': namespace}.items(),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=namespace,
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=namespace,
            condition=UnlessCondition(LaunchConfiguration('gui')),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'm20.rviz')],
            output='log',
        ),
    ])
