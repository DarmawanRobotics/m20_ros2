"""In-process tests of the driver against the simulated M20 on the loopback interface."""

import itertools
import math
import os
import threading
import time

from fake_robot import FakeM20, FakeServer
from m20_driver import protocol as p
import pytest

rclpy = pytest.importorskip('rclpy')
Parameter = pytest.importorskip('rclpy.parameter').Parameter
MultiThreadedExecutor = pytest.importorskip('rclpy.executors').MultiThreadedExecutor
geometry_msgs = pytest.importorskip('geometry_msgs.msg')
Odometry = pytest.importorskip('nav_msgs.msg').Odometry
sensor_msgs = pytest.importorskip('sensor_msgs.msg')
diagnostic_msgs = pytest.importorskip('diagnostic_msgs.msg')
Trigger = pytest.importorskip('std_srvs.srv').Trigger
M20DriverNode = pytest.importorskip('m20_driver.driver_node').M20DriverNode

RIGS = itertools.count()


def wait_for(predicate, timeout: float = 5.0) -> bool:
    """Poll until the predicate holds."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def values_of(status) -> dict:
    """Return the KeyValues of a DiagnosticStatus as a dict."""
    return {kv.key: kv.value for kv in status.values}


class Rig:
    """A driver, a simulated robot and a listener, spinning on their own threads."""

    def __init__(self, robot: FakeM20, port=None, **overrides):
        """Start everything; port None serves the robot, else the driver talks to that port."""
        self.server = FakeServer(robot, rate_scale=2.0) if port is None else None
        self.robot = robot
        params = {
            'robot_ip': '127.0.0.1',
            'robot_port': self.server.port if self.server else port,
            'local_ip': '127.0.0.1',
            'local_port': 0,
            'heartbeat_hz': 10.0,
            'stand_timeout_sec': 5.0,
            'rl_timeout_sec': 3.0,
            'response_timeout_sec': 1.0,
        }
        params.update(overrides)
        namespace = f'm20_test_{os.getpid()}_{next(RIGS)}'     # no cross-talk between tests
        self.node = M20DriverNode(
            namespace=namespace,
            parameter_overrides=[Parameter(k, value=v) for k, v in params.items()],
        )
        self.listener = rclpy.create_node('m20_test_listener', namespace=namespace)
        self.got = {k: [] for k in ('odom', 'imu', 'joints', 'battery', 'gps', 'state', 'diag')}
        sub = self.listener.create_subscription
        sub(Odometry, 'odom', self.got['odom'].append, 10)
        sub(sensor_msgs.Imu, 'imu', self.got['imu'].append, 10)
        sub(sensor_msgs.JointState, 'joint_states', self.got['joints'].append, 10)
        sub(sensor_msgs.BatteryState, 'battery', self.got['battery'].append, 10)
        sub(sensor_msgs.NavSatFix, 'gps/fix', self.got['gps'].append, 10)
        sub(diagnostic_msgs.DiagnosticStatus, 'state', self.got['state'].append, 10)
        sub(diagnostic_msgs.DiagnosticArray, 'diagnostics', self.got['diag'].append, 10)
        self.cmd = self.listener.create_publisher(geometry_msgs.Twist, 'cmd_vel', 10)
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def call(self, name: str, timeout: float = 15.0):
        """Call a Trigger service of the driver and return the response."""
        client = self.listener.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5.0), name
        future = client.call_async(Trigger.Request())
        assert wait_for(future.done, timeout), f'{name} did not answer'
        return future.result()

    def connected(self) -> bool:
        """Return True once the driver reports a connected robot."""
        return any(values_of(s).get('connected') == 'true' for s in self.got['state'])

    def state(self, **expected):
        """Return the latest state whose values include expected, None before one arrives."""
        for status in reversed(list(self.got['state'])):
            values = values_of(status)
            if all(values.get(k) == v for k, v in expected.items()):
                return status
        return None

    def close(self):
        """Stop everything."""
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.listener.destroy_node()
        self.node.destroy_node()
        if self.server is not None:
            self.server.close()


@pytest.fixture
def ros():
    """Initialise and shut down rclpy around a test."""
    rclpy.init()
    rigs = []
    yield rigs
    for rig in rigs:
        rig.close()
    rclpy.shutdown()


def start(rigs, robot=None, **kwargs) -> Rig:
    """Start a rig and register it for cleanup."""
    rig = Rig(robot or FakeM20(), **kwargs)
    rigs.append(rig)
    return rig


def test_reports_become_topics(ros):
    """Motion, device and basic reports show up as odom, imu, joints, battery, gps and state."""
    rig = start(ros)
    assert wait_for(lambda: all(rig.got[k] for k in ('odom', 'imu', 'joints', 'battery',
                                                     'gps')) and rig.connected())
    odom = rig.got['odom'][-1]
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link'
    imu = rig.got['imu'][-1]
    assert imu.header.frame_id == 'imu_link'
    assert imu.orientation.z == pytest.approx(math.sin(0.15), abs=1e-3)   # yaw 0.3 rad
    assert imu.linear_acceleration.z == pytest.approx(9.81)
    joints = rig.got['joints'][-1]
    assert list(joints.name) == list(p.JOINT_NAMES)
    assert joints.position[1] == pytest.approx(0.7) and joints.position[2] == pytest.approx(-1.4)
    assert all(math.isfinite(joints.velocity[i]) for i in p.WHEEL_INDICES)
    battery = rig.got['battery'][-1]
    assert battery.percentage == pytest.approx(0.85) and battery.voltage == pytest.approx(50.2)
    assert battery.power_supply_status == sensor_msgs.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
    gps = rig.got['gps'][-1]
    assert (gps.latitude, gps.longitude) == (pytest.approx(-6.2), pytest.approx(106.8))
    assert gps.status.status == sensor_msgs.NavSatStatus.STATUS_FIX
    assert wait_for(lambda: rig.state(connected='true', errors='1', gait='basic'))
    state = rig.state(connected='true', errors='1', gait='basic')
    values = values_of(state)
    assert values['motion_state'] == 'sit' and values['gait'] == 'basic'
    assert values['usage_mode'] == 'regular' and values['angle_unit'] == 'rad'
    assert state.level == diagnostic_msgs.DiagnosticStatus.WARN and '1 errors' in state.message
    assert wait_for(lambda: any(s.name == 'm20 errors' and 'LeftFrontHipX' in s.message
                                for d in rig.got['diag'] for s in d.status))


def test_degrees_are_detected(ros):
    """A robot that reports degrees: the driver notices and the IMU is right anyway."""
    rig = start(ros, FakeM20(angles_in_degrees=True))
    assert wait_for(lambda: rig.got['imu'] and rig.connected())
    assert rig.got['imu'][-1].orientation.z == pytest.approx(math.sin(0.15), abs=1e-3)
    assert wait_for(lambda: rig.state(connected='true', angle_unit='deg'))


def test_stand_up_goes_through_stand_to_rl_control(ros):
    """Stand (1), wait for it, then RL control (17); a second call has nothing to do."""
    rig = start(ros)
    assert wait_for(rig.connected)
    answer = rig.call('stand_up')
    assert answer.success, answer.message
    assert answer.message == 'standing in RL control'
    params = [items['MotionParam'] for items in rig.robot.requests_of(p.MOTION_STATE)]
    assert params == [p.MOTION_STAND, p.MOTION_RL_CONTROL]
    assert rig.call('stand_up').message == 'already in RL control'


def test_cmd_vel_streams_axis_then_zeros_then_silence(ros):
    """cmd_vel becomes regular mode and axis ratios; the watchdog stops the robot."""
    robot = FakeM20(motion_state=p.MOTION_RL_CONTROL)
    robot.usage_mode = p.USAGE_NAVIGATION
    rig = start(ros, robot)
    assert wait_for(rig.connected)
    twist = geometry_msgs.Twist()
    twist.linear.x, twist.linear.y, twist.angular.z = 0.5, 0.2, 0.3
    for _ in range(12):
        rig.cmd.publish(twist)
        time.sleep(0.05)
    assert wait_for(lambda: robot.usage_mode == p.USAGE_REGULAR)
    assert {'Mode': p.USAGE_REGULAR} in robot.requests_of(p.USAGE_MODE)
    assert wait_for(lambda: any(items['X'] == 0.1 and items['Y'] == 0.1 and items['Yaw'] == 0.1
                                for items in robot.requests_of(p.AXIS)))
    time.sleep(0.8)
    assert robot.requests_of(p.AXIS)[-1] == p.axis_items()      # watchdog zeros
    time.sleep(0.6)
    count = len(robot.requests_of(p.AXIS))
    time.sleep(0.5)
    assert len(robot.requests_of(p.AXIS)) == count             # then silence
    assert wait_for(lambda: rig.got['odom'] and any(o.twist.twist.linear.x > 0.0
                                                    for o in rig.got['odom']))


def test_direction_back_keeps_cmd_vel_in_base_link(ros):
    """Forward direction Back: X and Y go out negated, the yaw rate does not."""
    robot = FakeM20(motion_state=p.MOTION_RL_CONTROL)
    robot.direction = 1
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', direction='back'))
    twist = geometry_msgs.Twist()
    twist.linear.x, twist.linear.y, twist.angular.z = 0.5, 0.2, 0.3
    for _ in range(6):
        rig.cmd.publish(twist)
        time.sleep(0.05)
    assert wait_for(lambda: any(items['X'] == -0.1 and items['Y'] == -0.1 and items['Yaw'] == 0.1
                                for items in robot.requests_of(p.AXIS)))


def test_plain_commands_and_refusals(ros):
    """Commands reach the robot; a refusal comes back as a failed call with the robot's code."""
    rig = start(ros)
    assert wait_for(rig.connected)
    assert rig.call('lights_on').success and rig.robot.front_light == 1
    assert rig.call('gait_flat_agile').success and rig.robot.gait == p.GAIT_FLAT_AGILE
    rig.robot.reject[p.CHARGE] = 0xE00A
    answer = rig.call('charge_start')
    assert not answer.success and '0xE00A' in answer.message
    answer = rig.call('sleep')
    assert answer.success and rig.robot.sleep == 1
    assert rig.robot.requests_of(p.SET_SLEEP)[-1] == {'Sleep': True, 'Auto': True, 'Time': 10}
    assert rig.call('wake').success and rig.robot.sleep == 0
    assert rig.call('sync_time').success and rig.robot.synced_time
    assert rig.call('emergency_stop').success
    assert rig.robot.motion_state == p.MOTION_SOFT_ESTOP


def test_without_a_robot(ros):
    """Nobody answers: stand_up refuses, the state says so."""
    rig = start(ros, port=9)   # discard port: nothing answers
    assert not rig.call('stand_up').success
    assert wait_for(lambda: rig.state(connected='false'))
    state = rig.state(connected='false')
    assert state.level == diagnostic_msgs.DiagnosticStatus.ERROR
    assert state.message == 'no reports from the robot'
