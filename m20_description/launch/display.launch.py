#!/usr/bin/env python3
"""The M20 URDF in RViz, with sliders for the joints (no robot needed)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    """Start the description, a joint state publisher and RViz in the namespace."""
    share = get_package_share_directory('m20_description')
    ns = LaunchConfiguration('namespace').perform(context).strip().strip('/') or 'm20'
    gui = LaunchConfiguration('gui')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'description.launch.py')),
            launch_arguments={'namespace': ns}.items(),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=ns,
            condition=IfCondition(gui),
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            namespace=ns,
            condition=UnlessCondition(gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(share, 'rviz', 'm20.rviz')],
            # the config shows /m20/robot_description: follow the namespace
            remappings=[('/m20/robot_description', f'/{ns}/robot_description')],
            output='log',
        ),
    ]


def generate_launch_description():
    """Declare the namespace and the sliders switch."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='m20'),
        DeclareLaunchArgument('gui', default_value='true', description='joint sliders'),
        OpaqueFunction(function=launch_setup),
    ])
