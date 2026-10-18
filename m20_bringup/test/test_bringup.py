"""The bringup launch file: its arguments and what it starts."""

import importlib.util
from pathlib import Path

import pytest

launch = pytest.importorskip('launch')
pytest.importorskip('launch_ros')
pytest.importorskip('ament_index_python')

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch.py'


def load():
    """Import the launch file as a module."""
    spec = importlib.util.spec_from_file_location('m20_bringup_launch', str(LAUNCH_FILE))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def actions_for(**args) -> list:
    """Return the actions launch_setup builds for the default arguments plus these."""
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    module = load()
    context = launch.LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = perform_substitutions(
                context, entity.default_value)
    context.launch_configurations.update(args)
    return module.launch_setup(context)


def nodes(actions) -> list:
    """Return (package, executable) of every Node action."""
    from launch_ros.actions import Node

    return [(a.node_package, a.node_executable) for a in actions if isinstance(a, Node)]


def test_arguments():
    """namespace, robot_ip, driver_params and rviz, with their defaults."""
    from launch.actions import DeclareLaunchArgument

    declared = {e.name for e in load().generate_launch_description().entities
                if isinstance(e, DeclareLaunchArgument)}
    assert declared == {'namespace', 'robot_ip', 'driver_params', 'rviz'}


def test_starts_the_driver_and_the_description():
    """The driver node, the description launch; RViz only on request."""
    from launch.actions import IncludeLaunchDescription

    actions = actions_for()
    assert nodes(actions) == [('m20_driver', 'driver_node')]
    assert any(isinstance(a, IncludeLaunchDescription) for a in actions)
    assert ('rviz2', 'rviz2') in nodes(actions_for(rviz='true'))


def test_robot_ip_overrides_the_params_file():
    """An empty robot_ip leaves the params file alone."""
    module = load()
    assert module.driver_parameters('p.yaml', '') == ['p.yaml']
    expected = ['p.yaml', {'robot_ip': '127.0.0.1'}]
    assert module.driver_parameters('p.yaml', ' 127.0.0.1 ') == expected
