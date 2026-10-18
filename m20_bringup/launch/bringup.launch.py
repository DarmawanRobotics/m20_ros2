#!/usr/bin/env python3
"""
The Lynx M20 in one launch: the driver and robot_state_publisher with the URDF.

    ros2 launch m20_bringup bringup.launch.py                       # namespace m20, 10.21.31.103
    ros2 launch m20_bringup bringup.launch.py robot_ip:=127.0.0.1   # against the fake robot
    ros2 launch m20_bringup bringup.launch.py rviz:=true

Both run in the robot namespace: the driver's joint_states drive robot_state_publisher there.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def share(package: str, *path: str) -> str:
    """Return a file in the installed share directory of a package."""
    return os.path.join(get_package_share_directory(package), *path)


def truthy(value: str) -> bool:
    """Read a launch argument as a bool."""
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def driver_parameters(params_file: str, robot_ip: str) -> list:
    """Return the driver's parameters: the file, and robot_ip when one was given."""
    robot_ip = robot_ip.strip()
    return [params_file] + ([{'robot_ip': robot_ip}] if robot_ip else [])


def launch_setup(context):
    """Build the actions from the arguments."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    ns = arg('namespace').strip().strip('/') or 'm20'
    robot_ip = arg('robot_ip').strip()
    actions = [
        LogInfo(msg=f'M20 namespace={ns} robot_ip={robot_ip or "(driver params)"}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                share('m20_description', 'launch', 'description.launch.py')
            ),
            launch_arguments={'namespace': ns}.items(),
        ),
        Node(
            package='m20_driver',
            executable='driver_node',
            name='m20_driver',
            namespace=ns,
            parameters=driver_parameters(arg('driver_params'), robot_ip),
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
    ]
    if truthy(arg('rviz')):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', share('m20_description', 'rviz', 'm20.rviz')],
            # the config shows /m20/robot_description
            remappings=[('/m20/robot_description', f'/{ns}/robot_description')],
            output='log',
        ))
    return actions


def generate_launch_description():
    """Declare the arguments."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='m20', description='driver topics/services and the URDF'
        ),
        DeclareLaunchArgument(
            'robot_ip', default_value='',
            description='the robot (UDP server); empty: driver_params (10.21.31.103)',
        ),
        DeclareLaunchArgument(
            'driver_params', default_value=share('m20_driver', 'config', 'driver_params.yaml'),
            description='a copy with the settings of this robot (speeds, signs, frames)',
        ),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz with the model'),
        OpaqueFunction(function=launch_setup),
    ])
