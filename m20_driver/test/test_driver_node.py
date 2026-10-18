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
RL = p.MOTION_RL_CONTROL


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


def twist(x: float = 0.0, y: float = 0.0, wz: float = 0.0):
    """Return a Twist."""
    msg = geometry_msgs.Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = x, y, wz
    return msg


def yaw_of(q) -> float:
    """Return the yaw of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Rig:
    """A driver, a simulated robot and a listener, spinning on their own threads."""

    def __init__(self, robot: FakeM20, port=None, respond=True, **overrides):
        """Start everything; port None serves the robot, else the driver talks to that port."""
        self.server = (FakeServer(robot, rate_scale=2.0, respond=respond)
                       if port is None else None)
        self.robot = robot
        params = {
            'robot_ip': '127.0.0.1',
            'robot_port': self.server.port if self.server else port,
            'local_ip': '127.0.0.1',
            'local_port': 0,
            'heartbeat_hz': 10.0,
            'stand_timeout_sec': 5.0,
            'rl_timeout_sec': 3.0,
            'transition_timeout_sec': 3.0,
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
        self.executor = MultiThreadedExecutor(num_threads=5)
        self.executor.add_node(self.node)
        self.executor.add_node(self.listener)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def call(self, name: str, timeout: float = 20.0):
        """Call a Trigger service of the driver and return the response."""
        client = self.listener.create_client(Trigger, name)
        assert client.wait_for_service(timeout_sec=5.0), name
        future = client.call_async(Trigger.Request())
        assert wait_for(future.done, timeout), f'{name} did not answer'
        return future.result()

    def stream(self, msg, seconds: float, rate: float = 10.0):
        """Publish a Twist at rate for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd.publish(msg)
            time.sleep(1.0 / rate)

    def stream_in_background(self, msg, seconds: float) -> threading.Thread:
        """Publish a Twist at 10 Hz on a thread; return it (join it)."""
        thread = threading.Thread(target=self.stream, args=(msg, seconds))
        thread.start()
        return thread

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

    def latest(self) -> dict:
        """Return the values of the latest state."""
        return values_of(self.got['state'][-1]) if self.got['state'] else {}

    def x_since(self, start: float) -> list:
        """Return the X of every axis command the robot received since start."""
        return [items['X'] for items in self.robot.since(start, p.AXIS)]

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


def driving_rig(ros, **kwargs) -> Rig:
    """Return a rig whose robot is in RL control and regular mode, reports arriving."""
    rig = start(ros, FakeM20(motion_state=RL), **kwargs)
    assert wait_for(lambda: rig.state(connected='true', usage_mode='regular'))
    return rig


# ---------------------------------------------------------------- reports


def test_reports_become_topics(ros):
    """Motion, device and basic reports show up as odom, imu, joints, battery, gps and state."""
    rig = start(ros, angle_unit='rad')
    assert wait_for(lambda: all(rig.got[k] for k in ('odom', 'imu', 'joints', 'battery',
                                                     'gps')) and rig.connected())
    odom = rig.got['odom'][-1]
    assert odom.header.frame_id == 'odom' and odom.child_frame_id == 'base_link'
    imu = rig.got['imu'][-1]
    assert imu.header.frame_id == 'imu_link'
    assert imu.orientation.z == pytest.approx(math.sin(0.15), abs=1e-3)   # yaw 0.3 rad
    assert imu.orientation_covariance[0] == pytest.approx(0.01)
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
    assert values['motion_state'] == 'sit' and values['velocity'] == 'idle'
    assert values['usage_mode'] == 'regular' and values['angle_unit'] == 'rad'
    assert state.level == diagnostic_msgs.DiagnosticStatus.WARN and '1 errors' in state.message
    assert wait_for(lambda: any(s.name == 'm20 errors' and 'LeftFrontHipX' in s.message
                                for d in rig.got['diag'] for s in d.status))


def test_small_angles_in_degrees_are_found_from_the_yaw_rate(ros):
    """Yaw 2.86 (degrees): no orientation until a turn shows degrees, then the right yaw."""
    robot = FakeM20(motion_state=RL, angles_in_degrees=True)
    robot.yaw = 0.05
    rig = start(ros, robot)
    assert wait_for(lambda: rig.got['imu'] and rig.connected())
    assert rig.got['imu'][-1].orientation_covariance[0] == -1.0
    assert rig.latest().get('angle_unit') == 'unknown (auto)'
    rig.stream(twist(wz=1.0), 4.0)                  # OmegaZ 0.5 rad/s in the simulator
    assert wait_for(lambda: rig.state(angle_unit='deg (auto)'))
    imu = rig.got['imu'][-1]
    assert imu.orientation_covariance[0] == pytest.approx(0.01)
    assert yaw_of(imu.orientation) == pytest.approx(math.remainder(robot.yaw, 2 * math.pi),
                                                    abs=0.05)


def test_unwrapped_radians_stay_radians(ros):
    """Yaw 7.0 rad (more than a turn) does not make degrees: the turn finds radians."""
    robot = FakeM20(motion_state=RL)
    robot.yaw = 7.0
    rig = start(ros, robot)
    assert wait_for(rig.connected)
    rig.stream(twist(wz=1.0), 4.0)
    assert wait_for(lambda: rig.state(angle_unit='rad (auto)'))
    yaw = yaw_of(rig.got['imu'][-1].orientation)
    assert yaw == pytest.approx(math.remainder(robot.yaw, 2 * math.pi), abs=0.05)


def test_invalid_parameters_do_not_start_the_driver(ros):
    """cmd_rate_hz 0, a zero full scale, response_timeout_sec 0: ERROR state, robot untouched."""
    for override in ({'cmd_rate_hz': 0.0}, {'full_scale.basic.vx': 0.0},
                     {'response_timeout_sec': 0.0}, {'robot_ip': '10.21.31.300'}):
        rig = start(ros, **override)
        assert rig.node.invalid, override
        assert wait_for(lambda: rig.got['state'])
        state = rig.got['state'][-1]
        assert state.level == diagnostic_msgs.DiagnosticStatus.ERROR
        assert state.message.startswith('invalid parameters')
        assert rig.server.heartbeats == 0 and rig.robot.requests == []


# ---------------------------------------------------------------- services


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


def test_stand_up_fails_fast_on_hard_estop_or_charging(ros):
    """HES or a charging state: refused at once, nothing sent."""
    robot = FakeM20()
    robot.hes = 1
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', hard_estop='triggered'))
    answer = rig.call('stand_up')
    assert not answer.success and 'hard emergency stop' in answer.message
    robot.hes, robot.charge = 0, 2
    assert wait_for(lambda: rig.state(charge='charging'))
    answer = rig.call('stand_up')
    assert not answer.success and 'charging' in answer.message
    assert robot.requests_of(p.MOTION_STATE) == []


def test_plain_commands_and_refusals(ros):
    """Commands reach the robot; a refusal comes back as a failed call with the robot's code."""
    rig = driving_rig(ros)
    assert rig.call('lights_on').success and rig.robot.front_light == 1
    assert rig.call('gait_flat_agile').success and rig.robot.gait == p.GAIT_FLAT_AGILE
    rig.robot.reject[p.CHARGE] = 0xE00A
    answer = rig.call('charge_start')
    assert not answer.success and '0xE00A' in answer.message
    queries = len(rig.robot.requests_of(p.GET_SLEEP))
    answer = rig.call('sleep')
    assert answer.success and rig.robot.sleep == 1
    assert rig.robot.requests_of(p.SET_SLEEP)[-1] == {'Sleep': True, 'Auto': True, 'Time': 10}
    assert rig.call('wake').success and rig.robot.sleep == 0
    assert len(rig.robot.requests_of(p.GET_SLEEP)) == queries + 2     # read before each
    assert rig.call('sync_time').success and rig.robot.synced_time


def test_gaits_need_rl_control_regular_mode_and_both_batteries(ros):
    """Gait switching is refused outside RL control, in another mode or on one battery."""
    rig = start(ros)
    assert wait_for(rig.connected)
    answer = rig.call('gait_stair')
    assert not answer.success and 'RL control' in answer.message
    rig.robot.motion_state = RL
    rig.robot.power_management = 1
    assert wait_for(lambda: rig.state(motion_state='rl_control',
                                      power_management='single_battery'))
    assert 'single battery' in rig.call('gait_stair').message
    assert rig.robot.requests_of(p.GAIT) == []


def test_motion_states_are_confirmed_by_the_report(ros):
    """sit_down succeeds when Sit is reported; idle and damping only from a sitting state."""
    rig = driving_rig(ros)
    answer = rig.call('idle')
    assert not answer.success and 'sit down first' in answer.message
    answer = rig.call('sit_down')
    assert answer.success and answer.message == 'sit'
    answer = rig.call('damping')
    assert answer.success and rig.robot.motion_state == p.MOTION_DAMPING
    assert rig.state(velocity='hold:damping+sit_down') or wait_for(
        lambda: rig.state(velocity='hold:damping+sit_down'))


class IgnoresSit(FakeM20):
    """Answers Sit with success but stays where it is."""

    def handle(self, frame, now):
        """Pretend to sit down."""
        if frame.key == p.MOTION_STATE and frame.items.get('MotionParam') == p.MOTION_SIT:
            self.requests.append((frame.kind, frame.command, frame.items))
            return 0
        return super().handle(frame, now)


def test_sit_down_fails_when_the_state_never_comes(ros):
    """A reply alone is not enough for a motion state: the report has to show it."""
    rig = start(ros, IgnoresSit(motion_state=RL), transition_timeout_sec=1.0)
    assert wait_for(rig.connected)
    answer = rig.call('sit_down')
    assert not answer.success and answer.message.startswith('not sit after')


def test_release_emergency_stop_and_charge_cancel(ros):
    """Release with MotionParam -2 leaves the soft e-stop; Charge 3 cancels docking."""
    robot = FakeM20(motion_state=p.MOTION_SOFT_ESTOP)
    robot.charge = 1
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', motion_state='soft_emergency_stop'))
    answer = rig.call('release_emergency_stop')
    assert answer.success and robot.motion_state != p.MOTION_SOFT_ESTOP
    assert robot.requests_of(p.MOTION_STATE)[-1] == {'MotionParam': -2}
    assert rig.call('charge_cancel').success and robot.charge == 0


class DropsEstop(FakeM20):
    """The soft e-stop request is lost on the way (UDP)."""

    def handle(self, frame, now):
        """Drop MotionParam 2."""
        if frame.key == p.MOTION_STATE and frame.items.get('MotionParam') == p.MOTION_SOFT_ESTOP:
            return 0
        return super().handle(frame, now)


def test_emergency_stop_is_not_confirmed_without_reply_or_state(ros):
    """Lost request, no call status: success False and NOT confirmed (was: success)."""
    rig = start(ros, DropsEstop(motion_state=RL), respond=False)
    assert wait_for(rig.connected)
    answer = rig.call('emergency_stop')
    assert not answer.success and 'NOT confirmed' in answer.message
    assert len(rig.robot.requests_of(p.MOTION_STATE)) == 0      # dropped, all three


def test_emergency_stop_confirmed_by_the_state_without_reply(ros):
    """No call status, but the report shows soft_emergency_stop: confirmed, sent three times."""
    rig = start(ros, FakeM20(motion_state=RL), respond=False)
    assert wait_for(rig.connected)
    answer = rig.call('emergency_stop')
    assert answer.success and 'soft_emergency_stop' in answer.message
    assert rig.robot.requests_of(p.MOTION_STATE) == [{'MotionParam': 2}] * 3


def test_services_fail_without_a_robot(ros):
    """Nobody answers: every command is unconfirmed (was: success)."""
    rig = start(ros, port=9, response_timeout_sec=0.3, transition_timeout_sec=0.5)
    for name in ('emergency_stop', 'sit_down', 'damping', 'charge_start', 'navigation_mode',
                 'lights_on', 'stand_up'):
        answer = rig.call(name)
        assert not answer.success, (name, answer.message)
    assert 'unconfirmed: no reply' in rig.call('lights_off').message
    assert wait_for(lambda: rig.state(connected='false'))
    state = rig.state(connected='false')
    assert state.level == diagnostic_msgs.DiagnosticStatus.ERROR
    assert state.message == 'no reports from the robot'


def test_refusal_with_another_reply_id_fails(ros):
    """The robot refuses and answers with message ID 0: still a failed call (was: success)."""
    robot = FakeM20()
    robot.reject[p.CHARGE] = 0xE00A
    rig = start(ros, robot)
    rig.server.reply_id = 0
    assert wait_for(rig.connected)
    answer = rig.call('charge_start')
    assert not answer.success and '0xE00A' in answer.message


# ---------------------------------------------------------------- velocity gate


def test_cmd_vel_switches_from_navigation_once_drives_then_bursts(ros):
    """Navigation since boot: regular mode requested, axis ratios, zeros, then silence."""
    robot = FakeM20(motion_state=RL)
    robot.usage_mode = p.USAGE_NAVIGATION
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', usage_mode='navigation'))
    rig.stream(twist(0.5, 0.2, 0.3), 1.5)
    assert robot.usage_mode == p.USAGE_REGULAR
    assert {'Mode': p.USAGE_REGULAR} in robot.requests_of(p.USAGE_MODE)
    assert any(items['X'] == 0.1 and items['Y'] == 0.1 and items['Yaw'] == 0.1
               for items in robot.requests_of(p.AXIS))
    stop = time.monotonic()
    time.sleep(1.5)
    after = rig.x_since(stop)
    moving = [x for x in after if x != 0.0]
    assert after and after[-1] == 0.0 and len(moving) <= 12          # stale after 0.5 s
    assert len(after) - len(moving) <= 12                             # one burst of zeros
    count = len(robot.requests_of(p.AXIS))
    time.sleep(0.5)
    assert len(robot.requests_of(p.AXIS)) == count                    # then silence
    assert any(o.twist.twist.linear.x > 0.0 for o in rig.got['odom'])


def test_zero_twists_keep_the_axis_channel_free(ros):
    """4 s of zero Twists send no axis command at all (was: 50 zeros)."""
    rig = driving_rig(ros)
    begin = time.monotonic()
    rig.stream(twist(), 4.0)
    assert rig.x_since(begin) == []


def test_noise_level_cmd_vel_requests_no_mode(ros):
    """0.002 m/s rounds to 0.00: no motion, no regular mode request (was: a request)."""
    robot = FakeM20(motion_state=RL)
    robot.usage_mode = p.USAGE_NAVIGATION
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', usage_mode='navigation'))
    begin = time.monotonic()
    rig.stream(twist(0.002), 1.0)
    assert robot.since(begin, p.USAGE_MODE) == [] and robot.since(begin, p.AXIS) == []


def test_nan_in_one_component_stops(ros):
    """Twist(x=0.5, y=NaN) is a stop, X is never sent (was: X 0.1)."""
    rig = driving_rig(ros)
    begin = time.monotonic()
    rig.stream(twist(0.5, float('nan'), 0.0), 0.8)
    assert [x for x in rig.x_since(begin) if x != 0.0] == []


def test_emergency_stop_holds_a_running_stream(ros):
    """After emergency_stop the stream drives no more (was: 37 axis commands with X 0.1)."""
    rig = driving_rig(ros)
    thread = rig.stream_in_background(twist(0.5), 4.0)
    time.sleep(1.0)
    answer = rig.call('emergency_stop')
    begin = time.monotonic()
    thread.join()
    assert answer.success, answer.message
    assert [x for x in rig.x_since(begin) if x != 0.0] == []
    assert wait_for(lambda: rig.state(velocity='hold:emergency_stop'))


def test_stand_up_after_estop_waits_for_a_zero(ros):
    """Soft e-stop with a stale stream, then stand_up: no motion until cmd_vel was zero."""
    rig = start(ros, FakeM20(motion_state=p.MOTION_SOFT_ESTOP))
    assert wait_for(lambda: rig.state(connected='true', motion_state='soft_emergency_stop'))
    thread = rig.stream_in_background(twist(0.5), 5.0)
    time.sleep(0.5)
    answer = rig.call('stand_up')
    begin = time.monotonic()
    thread.join()
    assert answer.success and rig.robot.motion_state == RL
    assert [x for x in rig.x_since(begin) if x != 0.0] == []
    rig.stream(twist(), 0.3)
    begin = time.monotonic()
    rig.stream(twist(0.5), 0.8)
    assert 0.1 in rig.x_since(begin)


def test_takeover_latches_until_regular_mode_is_reported(ros):
    """The app switches to navigation: no axis, no mode request, through zeros and silence."""
    rig = driving_rig(ros)
    robot = rig.robot
    thread = rig.stream_in_background(twist(0.5), 4.0)
    time.sleep(1.0)
    begin = time.monotonic()
    robot.usage_mode = p.USAGE_NAVIGATION
    thread.join()
    assert robot.since(begin, p.USAGE_MODE) == []
    assert robot.usage_mode == p.USAGE_NAVIGATION
    late = time.monotonic() - 1.0
    assert [x for x in rig.x_since(late) if x != 0.0] == []
    state = rig.latest()
    assert state['velocity'].startswith('takeover (navigation mode)')
    rig.stream(twist(), 2.0)                                     # zeros
    time.sleep(2.0)                                              # silence
    begin = time.monotonic()
    rig.stream(twist(0.5), 1.0)                                  # a new episode
    assert robot.since(begin, p.USAGE_MODE) == [] and rig.x_since(begin) == []
    robot.usage_mode = p.USAGE_REGULAR                           # handed back in the app
    thread = rig.stream_in_background(twist(0.5), 1.5)
    begin = time.monotonic()
    thread.join()
    assert [x for x in rig.x_since(begin) if x != 0.0] == []     # stale stream: zero first
    rig.stream(twist(), 0.3)
    begin = time.monotonic()
    rig.stream(twist(0.5), 0.8)
    assert 0.1 in rig.x_since(begin)


def test_takeover_while_cmd_vel_is_zero_latches(ros):
    """Leaving regular mode latches even when nothing moves."""
    rig = driving_rig(ros)
    rig.robot.usage_mode = p.USAGE_ASSIST
    assert wait_for(lambda: rig.latest().get('velocity', '').startswith('takeover (assist'))
    state = rig.got['state'][-1]
    assert state.level == diagnostic_msgs.DiagnosticStatus.WARN


class OtherClientDriving(FakeM20):
    """Another client holds the axis channel."""

    def __init__(self, *args, **kwargs):
        """Refuse every axis command with E006."""
        super().__init__(*args, **kwargs)
        self.axis_owner_other = True


def test_e006_yields_and_shows_in_state(ros):
    """E006: the driver goes quiet after the burst and says so (was: 12.5 Hz, nothing shown)."""
    rig = start(ros, OtherClientDriving(motion_state=RL))
    assert wait_for(lambda: rig.state(connected='true', usage_mode='regular'))
    begin = time.monotonic()
    rig.stream(twist(0.5), 2.0)
    assert len(rig.robot.since(begin, p.AXIS)) <= 16          # a few, then the zero burst
    assert wait_for(lambda: rig.state(velocity='axis_owner_other'))
    state = rig.state(velocity='axis_owner_other')
    assert state.level == diagnostic_msgs.DiagnosticStatus.WARN and 'E006' in state.message
    assert '0xE006' in values_of(state)['axis_refusal']


def test_stale_basic_report_keeps_the_direction(ros):
    """Direction=Back, basic reports stop: X stays negated (was: flipped to +0.1)."""
    robot = FakeM20(motion_state=RL)
    robot.direction = 1
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', direction='back'))
    rig.server.RATES = {k: v for k, v in FakeServer.RATES.items() if k != p.BASIC_STATUS}
    begin = time.monotonic()
    rig.stream(twist(0.5), 6.0)
    assert -0.1 in rig.x_since(begin) and 0.1 not in rig.x_since(begin)


def test_direction_back_keeps_cmd_vel_in_base_link(ros):
    """Forward direction Back: X and Y go out negated, the yaw rate does not."""
    robot = FakeM20(motion_state=RL)
    robot.direction = 1
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', direction='back'))
    rig.stream(twist(0.5, 0.2, 0.3), 0.6)
    assert any(items['X'] == -0.1 and items['Y'] == -0.1 and items['Yaw'] == 0.1
               for items in robot.requests_of(p.AXIS))


def test_watchdog_is_not_starved_by_blocking_services(ros):
    """Four services wait on a silent robot: the zeros still follow the stop within 0.7 s."""
    rig = driving_rig(ros, response_timeout_sec=3.0)
    rig.stream(twist(0.5), 1.0)
    rig.server.respond = False
    callers = [threading.Thread(target=rig.call, args=(name,))
               for name in ('lights_on', 'lights_off', 'gait_basic', 'gait_stair')]
    for caller in callers:
        caller.start()
    stop = time.monotonic()
    time.sleep(1.5)
    after = [(now - stop, items['X']) for now, key, items in list(rig.robot.log)
             if key == p.AXIS and now >= stop]
    first_zero = next((dt for dt, x in after if x == 0.0), None)
    assert first_zero is not None and first_zero < 0.7, after
    for caller in callers:
        caller.join()


def test_overspeed_disarms(ros):
    """The robot runs far faster than max_vx (wrong full scale): disarmed, nothing more."""
    robot = FakeM20(motion_state=RL)
    robot.FULL_SCALE = {'X': 20.0, 'Y': 1.0, 'Yaw': 1.5}      # 0.1 -> 2 m/s
    rig = start(ros, robot)
    assert wait_for(lambda: rig.state(connected='true', usage_mode='regular'))
    thread = rig.stream_in_background(twist(0.5), 3.0)
    assert wait_for(lambda: rig.state(velocity='disarmed:overspeed'), timeout=3.0)
    begin = time.monotonic()
    thread.join()
    assert [x for x in rig.x_since(begin) if x != 0.0] == []
