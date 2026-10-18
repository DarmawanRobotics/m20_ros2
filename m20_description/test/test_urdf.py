"""The M20 URDF: the official model with m20_driver's joint names, and its sensor frames."""

import importlib.util
from pathlib import Path
from xml.etree import ElementTree

import pytest

xacro = pytest.importorskip('xacro')

PACKAGE = Path(__file__).resolve().parents[1]
XACRO = PACKAGE / 'urdf' / 'm20.urdf.xacro'


def driver_protocol():
    """Return m20_driver.protocol: installed, else from the repository's source tree."""
    try:
        from m20_driver import protocol
        return protocol
    except ImportError:
        pass
    source = PACKAGE.parent / 'm20_driver' / 'm20_driver' / 'protocol.py'
    if not source.is_file():
        pytest.skip('m20_driver not found')
    spec = importlib.util.spec_from_file_location('m20_driver_protocol', str(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def root() -> ElementTree.Element:
    """Return the expanded URDF."""
    return ElementTree.fromstring(xacro.process_file(str(XACRO)).toxml())


def joint(root, name):
    """Return a joint element by name."""
    found = root.find(f"joint[@name='{name}']")
    assert found is not None, f'no joint {name}'
    return found


def origin(element) -> tuple:
    """Return the xyz of an origin element."""
    return tuple(float(v) for v in element.find('origin').get('xyz').split())


def test_joint_names_are_the_drivers(root):
    """The 16 moving joints are m20_driver's JOINT_NAMES; its wheel indices are the wheels."""
    protocol = driver_protocol()
    moving = [j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed']
    assert moving == list(protocol.JOINT_NAMES)
    continuous = {j.get('name') for j in root.findall("joint[@type='continuous']")}
    assert continuous == {protocol.JOINT_NAMES[i] for i in protocol.WHEEL_INDICES}


def test_legs_are_the_official_model(root):
    """Placement, axes and limits as in M20.urdf."""
    assert root.get('name') == 'm20'
    assert origin(joint(root, 'fl_hipx_joint')) == (0.3141, 0.0685, 0.0)
    assert origin(joint(root, 'hr_knee_joint')) == (0.0, -0.0984, -0.25)
    assert origin(joint(root, 'hr_wheel_joint')) == (0.0, -0.059676, -0.25)
    for name, axis in (('fl_hipx_joint', '-1 0 0'), ('fl_hipy_joint', '0 -1 0'),
                       ('hl_knee_joint', '0 -1 0'), ('fr_wheel_joint', '0 -1 0')):
        assert joint(root, name).find('axis').get('xyz') == axis
    limit = joint(root, 'fl_hipx_joint').find('limit')
    assert (float(limit.get('lower')), float(limit.get('upper'))) == (-0.436, 0.611)
    limit = joint(root, 'hl_hipy_joint').find('limit')
    assert (float(limit.get('lower')), float(limit.get('upper'))) == (-2.286, 2.583)
    mass = float(root.find("link[@name='base_link']/inertial/mass").get('value'))
    assert mass == 15.882


def test_meshes_are_in_the_package(root):
    """Every mesh the URDF names is installed with the package; every material is named."""
    files = {m.get('filename') for m in root.iter('mesh')}
    assert len(files) == 17
    for name in files:
        assert name.startswith('package://m20_description/meshes/')
        assert (PACKAGE / 'meshes' / name.rsplit('/', 1)[1]).is_file(), name
    assert all(m.get('name') for m in root.iter('material'))


def test_sensor_frames(root):
    """Cameras and LiDARs where the official model puts them; imu_link is base_link."""
    assert origin(joint(root, 'front_camera_joint')) == (0.37646, 0.0, 0.03738)
    assert origin(joint(root, 'rear_camera_joint')) == (-0.37646, 0.0, 0.03738)
    assert origin(joint(root, 'front_lidar_joint')) == (0.32028, 0.0, -0.013)
    assert origin(joint(root, 'rear_lidar_joint')) == (-0.32028, 0.0, -0.013)
    imu = joint(root, 'imu_joint')
    assert origin(imu) == (0.0, 0.0, 0.0)
    assert imu.find('origin').get('rpy') == '0 0 0'
    assert imu.find('parent').get('link') == 'base_link'


def test_one_tree_from_base_link(root):
    """Every link has one parent, except base_link (odom -> base_link comes from the driver)."""
    names = {link.get('name') for link in root.findall('link')}
    children = [j.find('child').get('link') for j in root.findall('joint')]
    assert len(children) == len(set(children))
    assert names - set(children) == {'base_link'}
    assert not names & {'odom', 'map'}
