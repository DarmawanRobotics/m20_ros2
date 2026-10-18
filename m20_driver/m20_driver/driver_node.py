#!/usr/bin/env python3
"""
ROS 2 driver of the Deep Robotics Lynx M20 over its inspection protocol (UDP, JSON).

cmd_vel goes through a velocity gate (holds, arming, takeover latch, E006 yield, watchdog) to
axis commands in regular usage mode; the robot's reports become odometry, IMU, joint states,
battery, GPS and a readable state. Every robot command is a std_srvs/Trigger service that
succeeds only when the robot confirmed it: its call status response, and for motion states the
state it then reports.
"""

import math
import socket
import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from m20_driver import protocol as p
from m20_driver.convert import (
    AngleUnitDetector,
    battery_summary,
    finite_or,
    gps_status,
    Odometry2D,
    quaternion_from_rpy,
    WheelAngles,
)
from m20_driver.link import RobotLink
from m20_driver.motion import (
    GAIT_NAMES,
    NAV_HOLDS,
    RobotView,
    SpeedEstimator,
    VelocityConfig,
    VelocityGate,
)
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger

try:  # Galactic and later
    from rclpy.executors import ExternalShutdownException
except ImportError:  # Foxy
    class ExternalShutdownException(Exception):
        """Placeholder: Foxy has no ExternalShutdownException."""

GPS_UERE = 4.0              # m, user range error that turns DOP into a rough standard deviation
CONFIG_ERROR_EXIT = 2
EXECUTOR_THREADS = 5        # e-stop, services, velocity, publishing: never starved
ESTOP_REPEATS = 3
ESTOP_SPACING_SEC = 0.05
ESTOP_CONFIRM_SEC = 1.5
HUNG_SEC = 1.0              # velocity loop silent this long: heartbeats pause
SIGN_CHECK_SEC = 2.0
SDK_MOVING = (0.25, 0.25, 1.0)   # the official client's isMoving: |LinearX|, |LinearY|, |OmegaZ|
SHUTDOWN_ZEROS = 3

GAIT_PARAMS = {name: gait for gait, name in GAIT_NAMES.items()}
GAIT_SERVICES = {f'gait_{name}': gait for gait, name in GAIT_NAMES.items()}
# Motion state services: name -> state they ask for and wait for
MOTION_SERVICES = {
    'sit_down': p.MOTION_SIT,
    'idle': p.MOTION_IDLE,
    'damping': p.MOTION_DAMPING,
}
SITTING_STATES = (p.MOTION_SIT, p.MOTION_IDLE, p.MOTION_SOFT_ESTOP, p.MOTION_DAMPING)
# Plain commands: name -> (message, items, hold set before sending)
COMMANDS = {
    'navigation_mode': (p.USAGE_MODE, p.usage_mode_items(p.USAGE_NAVIGATION), 'navigation_mode'),
    'assist_mode': (p.USAGE_MODE, p.usage_mode_items(p.USAGE_ASSIST), 'assist_mode'),
    'lights_on': (p.LIGHTS, p.lights_items(True, True), None),
    'lights_off': (p.LIGHTS, p.lights_items(False, False), None),
    'charge_start': (p.CHARGE, p.charge_items(p.CHARGE_START), 'charge_start'),
    'charge_stop': (p.CHARGE, p.charge_items(p.CHARGE_STOP), None),
    'charge_cancel': (p.CHARGE, p.charge_items(p.CHARGE_CANCEL), None),   # from the SDK
}

FLOAT_PARAMS = {
    'heartbeat_hz': 5.0,
    'cmd_rate_hz': 20.0,
    'cmd_timeout_sec': 0.5,
    'stop_burst_sec': 0.5,
    'max_vx': 1.0,
    'max_vy': 0.5,
    'max_wz': 1.0,
    'state_hz': 2.0,
    'state_timeout_sec': 1.5,
    'response_timeout_sec': 1.0,
    'stand_timeout_sec': 10.0,
    'rl_timeout_sec': 5.0,
    'transition_timeout_sec': 10.0,
    'low_battery': 0.2,
}
POSITIVE = ('heartbeat_hz', 'cmd_rate_hz', 'cmd_timeout_sec', 'stop_burst_sec', 'max_vx',
            'max_vy', 'max_wz', 'state_hz', 'state_timeout_sec', 'response_timeout_sec',
            'stand_timeout_sec', 'rl_timeout_sec', 'transition_timeout_sec')
BOOL_PARAMS = {
    'auto_regular_mode': True,
    'invert_vy': False,
    'invert_wz': False,
    'follow_direction': True,
    'direction_flips_reports': False,
    'publish_tf': False,
    'sleep_auto': False,
}
STRING_PARAMS = {
    'robot_ip': p.ROBOT_IP,
    'local_ip': '0.0.0.0',
    'angle_unit': 'auto',
    'odom_frame': 'odom',
    'base_frame': 'base_link',
    'imu_frame': 'imu_link',
    'gps_frame': 'base_link',
    'stand_up_gait': '',
}
INT_PARAMS = {
    'robot_port': p.UDP_PORT,
    'local_port': 0,
    'sleep_minutes': 10,
}
FULL_SCALE_DEFAULTS = {'vx': 5.0, 'vy': 2.0, 'wz': 3.0}


def validate(v: dict):
    """Return (errors, warnings) of the parameter values; fixes the values that only warn."""
    errors, warnings = [], []
    for name in POSITIVE:
        value = v[name]
        if not (isinstance(value, float) and math.isfinite(value) and value > 0.0):
            errors.append(f'{name} must be a number above 0 (is {value!r})')
    for gait in GAIT_PARAMS:
        for axis in FULL_SCALE_DEFAULTS:
            name = f'full_scale.{gait}.{axis}'
            value = v[name]
            if not (isinstance(value, float) and math.isfinite(value) and value > 0.0):
                errors.append(f'{name} must be a number above 0 (is {value!r})')
    low = v['low_battery']
    if not (isinstance(low, float) and math.isfinite(low) and 0.0 <= low <= 1.0):
        errors.append(f'low_battery must be a number in [0, 1] (is {low!r})')
    for name in ('robot_ip', 'local_ip'):
        try:
            socket.inet_aton(str(v[name]))
        except OSError:
            errors.append(f'{name} {v[name]!r} is not an IPv4 address')
    for name, lowest in (('robot_port', 1), ('local_port', 0)):
        port = v[name]
        if not (isinstance(port, int) and not isinstance(port, bool) and lowest <= port <= 65535):
            errors.append(f'{name} {port!r} is not a port ({lowest}-65535)')
    minutes = v['sleep_minutes']
    if not (isinstance(minutes, int) and not isinstance(minutes, bool)):
        errors.append(f'sleep_minutes must be a whole number (is {minutes!r})')
    if v['angle_unit'] not in ('auto', 'rad', 'deg'):
        errors.append(f'angle_unit {v["angle_unit"]!r}: auto, rad or deg')
    if not errors:
        if v['cmd_timeout_sec'] < 2.0 / v['cmd_rate_hz']:
            errors.append(f'cmd_timeout_sec {v["cmd_timeout_sec"]} is shorter than two command '
                          f'periods ({2.0 / v["cmd_rate_hz"]:.3f} s at cmd_rate_hz)')
        if v['heartbeat_hz'] < 1.0:
            warnings.append(f'heartbeat_hz {v["heartbeat_hz"]} below the 1 Hz the document '
                            'asks for: using 1 Hz')
            v['heartbeat_hz'] = 1.0
        for axis, limit in (('vx', 'max_vx'), ('vy', 'max_vy'), ('wz', 'max_wz')):
            low = [g for g in GAIT_PARAMS if v[f'full_scale.{g}.{axis}'] < v[limit]]
            if low:
                warnings.append(f'{limit} {v[limit]} above full_scale.<gait>.{axis} of '
                                f'{", ".join(low)}: the axis saturates there')
    if not errors and not 5 <= v['sleep_minutes'] <= 30:
        clamped = max(5, min(30, v['sleep_minutes']))
        warnings.append(f'sleep_minutes {v["sleep_minutes"]} outside [5, 30]: using {clamped}')
        v['sleep_minutes'] = clamped
    gait = v['stand_up_gait'].strip()
    if gait and gait not in GAIT_PARAMS:
        warnings.append(f'stand_up_gait {gait!r} unknown ({", ".join(GAIT_PARAMS)}): ignored')
        v['stand_up_gait'] = ''
    return errors, warnings


class M20DriverNode(Node):
    """Bridges ROS 2 and the M20's inspection protocol."""

    def __init__(self, **kwargs):
        """Declare and check parameters, open the UDP link and create every interface."""
        super().__init__('m20_driver', **kwargs)
        v = self._read_parameters()
        errors, warnings = validate(v)
        for text in warnings:
            self.get_logger().warn(text)
        self.invalid = errors
        self._link = None
        self._state_pub = self.create_publisher(DiagnosticStatus, 'state', 10)
        if errors:
            for text in errors:
                self.get_logger().error(f'invalid parameter: {text}')
            self.get_logger().error('m20_driver does not start: fix the parameters (the robot is '
                                    'not contacted)')
            self.create_timer(1.0, self.publish_invalid)
            return

        self._v = v
        self._response_timeout = v['response_timeout_sec']
        self._stand_timeout = v['stand_timeout_sec']
        self._rl_timeout = v['rl_timeout_sec']
        self._transition_timeout = v['transition_timeout_sec']
        self._state_timeout = v['state_timeout_sec']
        self._low_battery = v['low_battery']
        self._direction_flips = v['direction_flips_reports']
        self._sleep_defaults = (v['sleep_auto'], v['sleep_minutes'])
        self._stand_up_gait = v['stand_up_gait'].strip()
        self._gate = VelocityGate(VelocityConfig(
            max_vx=v['max_vx'],
            max_vy=v['max_vy'],
            max_wz=v['max_wz'],
            full_scale={g: tuple(v[f'full_scale.{g}.{a}'] for a in ('vx', 'vy', 'wz'))
                        for g in GAIT_PARAMS},
            cmd_timeout_sec=v['cmd_timeout_sec'],
            stop_burst_sec=v['stop_burst_sec'],
            auto_regular_mode=v['auto_regular_mode'],
            invert_vy=v['invert_vy'],
            invert_wz=v['invert_wz'],
            follow_direction=v['follow_direction'],
        ))
        self._estimator = SpeedEstimator()
        self._angles = AngleUnitDetector(v['angle_unit'])
        self._odometry = Odometry2D()
        self._wheels = WheelAngles()

        self._data_lock = threading.Lock()
        self._axis_lock = threading.Lock()
        self._basic = None
        self._basic_time = None
        self._direction = None          # last reported, never aged out
        self._motion = None
        self._motion_time = None
        self._new_motion = False
        self._twist = (0.0, 0.0, 0.0)   # reported LinearX, LinearY, OmegaZ in base_link
        self._device = None
        self._new_device = False
        self._errors = ()
        self._sleep_settings = None
        self._sleep_event = threading.Event()
        self._axis_refusal = None       # (Response, time) of the last refused axis command
        self._refusal_warned = None
        self._sign_since = None
        self._sign_warning = False
        self._was_connected = False
        self._abort_seq = 0
        self._velocity_tick = time.monotonic()

        self._link = RobotLink(
            v['robot_ip'], v['robot_port'], v['local_ip'], v['local_port'], self.on_frame,
            heartbeat_hz=v['heartbeat_hz'],
            heartbeat_ok=lambda: time.monotonic() - self._velocity_tick <= HUNG_SEC,
            on_heartbeat_pause=self.on_heartbeat_pause,
        )

        self._tf = None
        if v['publish_tf']:
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu', 10)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self._gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        # cmd_vel and the velocity timer, the emergency stop and the other services each have
        # their own group: a blocking service never delays the watchdog or the emergency stop
        velocity = MutuallyExclusiveCallbackGroup()
        estop = MutuallyExclusiveCallbackGroup()
        services = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10, callback_group=velocity)
        self.create_timer(1.0 / v['cmd_rate_hz'], self.send_velocity, callback_group=velocity)
        self.create_service(Trigger, 'emergency_stop', self.service('emergency_stop',
                            self.emergency_stop), callback_group=estop)
        handlers = {
            'stand_up': self.stand_up,
            'rl_control': self.rl_control,
            'release_emergency_stop': self.release_emergency_stop,
            'regular_mode': self.regular_mode,
            'sleep': lambda seq: self.set_sleep(True, seq),
            'wake': lambda seq: self.set_sleep(False, seq),
            'sync_time': self.sync_time,
        }
        for name, state in MOTION_SERVICES.items():
            handlers[name] = self.motion_handler(name, state)
        for name, gait in GAIT_SERVICES.items():
            handlers[name] = self.gait_handler(name, gait)
        for name, (message, items, hold) in COMMANDS.items():
            handlers[name] = self.command_handler(name, message, items, hold)
        for name, handler in handlers.items():
            self.create_service(Trigger, name, self.service(name, handler),
                                callback_group=services)

        self.create_timer(0.5, self.watch_connection)
        self.create_timer(0.02, self.publish_reports)
        self.create_timer(1.0 / v['state_hz'], self.publish_state)
        self.create_timer(1.0, self.publish_diagnostics)

        self._link.start()
        self.get_logger().info(
            f'M20 at {self._link.remote[0]}:{self._link.remote[1]}, this end '
            f'{self._link.local[0]}:{self._link.local[1]} (the robot reports to the heartbeat '
            f'sender); angle unit {self._angles.name}'
        )

    def _read_parameters(self) -> dict:
        """Declare every parameter and return the values (floats as float, ints as int)."""
        v = {}
        for name, default in FLOAT_PARAMS.items():
            v[name] = self._number(name, default, float)
        for gait in GAIT_PARAMS:
            for axis, default in FULL_SCALE_DEFAULTS.items():
                name = f'full_scale.{gait}.{axis}'
                v[name] = self._number(name, default, float)
        for name, default in INT_PARAMS.items():
            v[name] = self._number(name, default, int)
        for name, default in BOOL_PARAMS.items():
            v[name] = bool(self.declare_parameter(name, default).value)
        for name, default in STRING_PARAMS.items():
            v[name] = str(self.declare_parameter(name, default).value)
        return v

    def _number(self, name: str, default, kind):
        """Declare a numeric parameter; return it as kind, or the raw value when it is not one."""
        value = self.declare_parameter(name, default).value
        if isinstance(value, bool):
            return value
        try:
            return kind(value)
        except (TypeError, ValueError):
            return value

    # ------------------------------------------------------------------ link

    def on_frame(self, frame: p.Frame):
        """Store one frame from the robot (receive thread)."""
        now = time.monotonic()
        key = frame.key
        if key == p.MOTION_STATUS:
            self._on_motion(p.decode_motion(frame.items), now)
        elif key == p.BASIC_STATUS:
            basic = p.decode_basic(frame.items)
            with self._data_lock:
                previous = self._direction
                self._basic = basic
                self._basic_time = now
                if basic.direction is not None:
                    self._direction = basic.direction
            if basic.direction == 1 and previous != 1:
                self.get_logger().warn(
                    'forward direction of the robot is Back (hard emergency stop side): '
                    + ('cmd_vel x and y are negated to stay in base_link'
                       if self._gate.config.follow_direction
                       else 'cmd_vel x and y act reversed (follow_direction is false)'))
        elif key == p.DEVICE_STATUS:
            device = p.decode_device(frame.items)
            with self._data_lock:
                self._device = device
                self._new_device = True
        elif key == p.ERROR_STATUS:
            errors = p.decode_errors(frame.items)
            with self._data_lock:
                changed = {e.code for e in errors} != {e.code for e in self._errors}
                self._errors = errors
            if changed and errors:
                self.get_logger().warn('robot errors: ' + '; '.join(e.describe() for e in errors))
        elif key == p.GET_SLEEP and 'Sleep' in frame.items:
            self._sleep_settings = (
                bool(frame.items.get('Auto', self._sleep_defaults[0])),
                int(finite_or(frame.items.get('Time'), self._sleep_defaults[1])),
            )
            self._sleep_event.set()
        elif key == p.AXIS:
            response = p.decode_response(frame)
            if response is not None and not response.ok:
                self._axis_refusal = (response, now)
                self._gate.axis_refused(response.error_code, now)
                if response.error_code != 0xE006 and (
                        self._refusal_warned is None or now - self._refusal_warned > 5.0):
                    self._refusal_warned = now
                    self.get_logger().warn(f'axis command refused: {response.describe()}')

    def _on_motion(self, motion, now: float):
        """Odometry, unit detection, overspeed and calibration data from one motion report."""
        vx, vy = finite_or(motion.linear_x, 0.0), finite_or(motion.linear_y, 0.0)
        wz = finite_or(motion.omega_z, 0.0)
        with self._data_lock:
            if self._direction_flips and self._direction == 1:
                vx, vy = -vx, -vy
            detected = self._angles.observe(now, motion.yaw, motion.omega_z)
            heading = self._angles.radians(motion.yaw)
            self._odometry.update(now, vx, vy, heading=heading,
                                  rate=wz if motion.omega_z is not None else None)
            if motion.joints is not None:
                self._wheels.update(now, [motion.joints[i] for i in p.WHEEL_INDICES])
            self._motion = motion
            self._motion_time = now
            self._new_motion = True
            self._twist = (vx, vy, wz)
            direction = self._direction
        if detected:
            self.get_logger().info(
                f'angle unit detected: {"degrees" if detected == "deg" else "radians"} (Yaw '
                f'against OmegaZ); set angle_unit: {detected} to have it from the first report')
        gate = self._gate
        gate.observe_speed(now, motion.linear_x, motion.linear_y, motion.omega_z)
        if gate.driving:
            gate_items = gate.last_axis
            self._estimator.update(gate.gait_name(), gate_items, (vx, vy, wz))
            self._check_sign(now, direction, gate_items, vx)
        else:
            self._sign_since = None

    def _check_sign(self, now: float, direction, items: dict, vx: float):
        """With Direction=Back: warn when the reported speed keeps the opposite sign."""
        sent = items.get('X', 0.0)
        if self._gate.config.follow_direction and direction == 1:
            sent = -sent                                   # back to base_link
        if direction != 1 or sent == 0.0 or abs(vx) < 0.05 or (vx > 0.0) == (sent > 0.0):
            self._sign_since = None
            return
        if self._sign_since is None:
            self._sign_since = now
        elif now - self._sign_since > SIGN_CHECK_SEC and not self._sign_warning:
            self._sign_warning = True
            self.get_logger().warn(
                'Direction=Back: the reported LinearX has the opposite sign of cmd_vel for '
                f'{SIGN_CHECK_SEC:.0f} s: check direction_flips_reports (and follow_direction)')

    def on_heartbeat_pause(self, paused: bool):
        """Log when the heartbeat pauses (velocity loop hung) or resumes (link thread)."""
        if paused:
            self.get_logger().warn(f'velocity loop silent for more than {HUNG_SEC:.0f} s: '
                                   'heartbeats paused (the robot loses this controller)')
        else:
            self.get_logger().info('velocity loop running again: heartbeats resumed')

    def report_age(self) -> float:
        """Return seconds since the last status report."""
        with self._data_lock:
            stamps = [t for t in (self._basic_time, self._motion_time) if t is not None]
        return time.monotonic() - max(stamps) if stamps else float('inf')

    def connected(self) -> bool:
        """Return True while status reports keep arriving."""
        return self.report_age() <= self._state_timeout

    def latest_basic(self):
        """Return the latest basic status (values are kept, never aged out), None before one."""
        with self._data_lock:
            return self._basic

    def motion_state(self):
        """Return the latest motion state (basic report, or the motion report when newer)."""
        with self._data_lock:
            basic, motion = self._basic, self._motion
            basic_time, motion_time = self._basic_time, self._motion_time
        if motion is not None and motion.motion_state is not None and (
            basic_time is None or (motion_time or 0.0) >= basic_time
        ):
            return motion.motion_state
        return basic.motion_state if basic is not None else None

    def gait(self):
        """Return the latest reported gait."""
        with self._data_lock:
            basic, motion = self._basic, self._motion
            basic_time, motion_time = self._basic_time, self._motion_time
        if motion is not None and motion.gait is not None and (
            basic_time is None or (motion_time or 0.0) >= basic_time
        ):
            return motion.gait
        return basic.gait if basic is not None else None

    def robot_view(self) -> RobotView:
        """Return what the gate needs; no motion state while reports are missing."""
        basic = self.latest_basic()
        with self._data_lock:
            direction = self._direction
        return RobotView(
            usage_mode=basic.usage_mode if basic else None,
            motion_state=self.motion_state() if self.connected() else None,
            hes=basic.hes if basic else None,
            charge=basic.charge if basic else None,
            direction=direction,
            gait=self.gait(),
        )

    # ------------------------------------------------------------------ motion

    def on_cmd_vel(self, msg: Twist):
        """Take a velocity command (base_link, REP-103); any NaN or infinity is a stop."""
        fields = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        finite = all(math.isfinite(value) for value in fields)
        self._gate.command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic(), finite)

    def send_velocity(self):
        """Send what the velocity gate decides for this period."""
        now = time.monotonic()
        self._velocity_tick = now
        view = self.robot_view()
        with self._axis_lock:
            for message, items in self._gate.tick(now, view):
                self._link.send(message, items)
        for level, text in self._gate.drain_events():
            # one call site per severity: rclpy refuses a severity change at the same line
            if level == 'error':
                self.get_logger().error(text)
            elif level == 'warn':
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)

    def watch_connection(self):
        """Log when reports start and stop; read the sleep settings on every (re)connect."""
        connected = self.connected()
        if connected and not self._was_connected:
            self.get_logger().info('M20 reports arriving')
            self._link.send(p.GET_SLEEP)
        elif not connected and self._was_connected:
            self.get_logger().warn('M20 reports stopped')
        self._was_connected = connected

    def walking(self) -> bool:
        """Return True while cmd_vel moved the robot within a second or it reports speed."""
        now = time.monotonic()
        if self._gate.recently_moved(now, 1.0):
            return True
        with self._data_lock:
            motion, stamp = self._motion, self._motion_time
        if motion is None or stamp is None or now - stamp > self._state_timeout:
            return False
        speeds = (motion.linear_x, motion.linear_y, motion.omega_z)
        return any(value is not None and math.isfinite(value) and abs(value) > limit
                   for value, limit in zip(speeds, SDK_MOVING))

    # ------------------------------------------------------------------ services

    def service(self, name: str, handler):
        """Return a Trigger callback that runs handler(abort_seq) -> (success, message)."""
        def callback(_request, response):
            seq = self._abort_seq
            try:
                ok, text = handler(seq)
            except Exception as error:  # noqa: B902  a service must always answer
                self.get_logger().error(f'{name} failed: {error!r}')
                ok, text = False, f'error: {error}'
            response.success, response.message = bool(ok), str(text)
            return response
        return callback

    def aborted(self, seq: int) -> bool:
        """Return True when emergency_stop was called since seq was taken."""
        return self._abort_seq != seq

    def call(self, message, items, done: str, seq: int):
        """Send a request; (True, done) only when the robot answered with ErrorCode 0."""
        sent, response = self._link.request(message, items, self._response_timeout,
                                            abort=lambda: self.aborted(seq))
        if not sent:
            return False, f'not sent: {self._link.last_error or "link closed"}'
        if response is None:
            if self.aborted(seq):
                return False, 'aborted by emergency_stop'
            return False, f'unconfirmed: no reply within {self._response_timeout:.1f} s'
        if not response.ok:
            return False, f'robot refused: {response.describe()}'
        return True, done

    def wait_until(self, predicate, timeout: float, seq: int):
        """Wait for predicate(); return True, False on timeout, or 'aborted'."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.aborted(seq):
                return 'aborted'
            if predicate():
                return True
            time.sleep(0.05)
        return bool(predicate())

    def wait_state(self, states, timeout: float, seq: int):
        """Wait until the reported motion state is one of states."""
        return self.wait_until(lambda: self.motion_state() in states, timeout, seq)

    def go_to(self, state: int, timeout: float, seq: int):
        """Ask for a motion state and wait for the report; (success, message)."""
        ok, text = self.call(p.MOTION_STATE, p.motion_state_items(state),
                             f'{p.name_of(p.MOTION_STATES, state)} sent', seq)
        if not ok:
            return False, text
        reached = self.wait_state((state,), timeout, seq)
        if reached == 'aborted':
            return False, 'aborted by emergency_stop'
        if not reached:
            return False, (f'not {p.name_of(p.MOTION_STATES, state)} after {timeout:.0f} s '
                           f'({p.name_of(p.MOTION_STATES, self.motion_state())})')
        return True, p.name_of(p.MOTION_STATES, state)

    def emergency_stop(self, seq: int):
        """Hold, zero, soft emergency stop three times; confirmed by a reply or the state."""
        self._abort_seq += 1
        now = time.monotonic()
        self._gate.hold('emergency_stop', now)
        with self._axis_lock:
            self._link.send(p.AXIS, p.axis_items())        # the timer keeps the zero burst going
        pending = []
        for i in range(ESTOP_REPEATS):
            request = self._link.submit(p.MOTION_STATE,
                                        p.motion_state_items(p.MOTION_SOFT_ESTOP))
            if request is not None:
                pending.append(request)
            if i < ESTOP_REPEATS - 1:
                time.sleep(ESTOP_SPACING_SEC)
        end = time.monotonic() + ESTOP_CONFIRM_SEC
        confirmed, refusal = None, None
        while confirmed is None and time.monotonic() < end:
            for request in pending:
                if request.response is not None:
                    if request.response.ok:
                        confirmed = 'robot replied'
                    else:
                        refusal = request.response
            if confirmed is None and self.motion_state() == p.MOTION_SOFT_ESTOP:
                confirmed = 'robot reports soft_emergency_stop'
            if confirmed is None:
                time.sleep(0.02)
        for request in pending:
            self._link.forget(request)
        if confirmed is not None:
            return True, f'soft emergency stop confirmed ({confirmed}); cmd_vel held'
        reason = (f'robot refused: {refusal.describe()}' if refusal is not None else
                  f'no reply and no soft_emergency_stop report within {ESTOP_CONFIRM_SEC} s')
        text = f'soft e-stop NOT confirmed: {reason}. Use the remote or the hard emergency stop'
        self.get_logger().error(text)
        return False, text

    def stand_up(self, seq: int):
        """Stand, then RL control; lifts every hold once RL control is reported."""
        basic = self.latest_basic()
        if not self.connected():
            return False, 'no reports from the robot'
        if basic is not None and basic.hes == 1:
            return False, 'hard emergency stop triggered: release it first'
        if basic is not None and basic.charge in p.CHARGING_STATES:
            return False, (f'charging ({p.name_of(p.CHARGE_STATES, basic.charge)}): '
                           'charge_stop or charge_cancel first')
        if self.motion_state() not in (p.MOTION_STAND, p.MOTION_RL_CONTROL):
            ok, text = self.call(p.MOTION_STATE, p.motion_state_items(p.MOTION_STAND),
                                 'stand sent', seq)
            if not ok:
                return False, text
            reached = self.wait_state((p.MOTION_STAND, p.MOTION_RL_CONTROL),
                                      self._stand_timeout, seq)
            if reached == 'aborted':
                return False, 'aborted by emergency_stop'
            if not reached:
                return False, (f'not standing after {self._stand_timeout:.0f} s '
                               f'({p.name_of(p.MOTION_STATES, self.motion_state())})')
        if self.motion_state() != p.MOTION_RL_CONTROL:
            ok, text = self.go_to(p.MOTION_RL_CONTROL, self._rl_timeout, seq)
            if not ok:
                return False, f'standing, but {text}'
            message = 'standing in RL control'
        else:
            message = 'already in RL control'
        self._gate.release()
        if self._stand_up_gait:
            ok, text = self.gait_handler(f'gait_{self._stand_up_gait}',
                                         GAIT_PARAMS[self._stand_up_gait])(seq)
            message += f', gait {self._stand_up_gait}' if ok else f', gait not set: {text}'
        return True, message

    def rl_control(self, seq: int):
        """RL control (from standing); lifts every hold once it is reported."""
        if self.motion_state() != p.MOTION_RL_CONTROL:
            ok, text = self.go_to(p.MOTION_RL_CONTROL, self._rl_timeout, seq)
            if not ok:
                return False, text
            message = 'rl_control'
        else:
            message = 'already in RL control'
        self._gate.release()
        return True, message

    def release_emergency_stop(self, seq: int):
        """Leave the soft emergency stop (MotionParam -2, from the SDK, not in V1.3.0)."""
        state = self.motion_state()
        if state is None or not self.connected():
            return False, 'no reports from the robot'
        if state != p.MOTION_SOFT_ESTOP:
            return True, f'not in soft emergency stop ({p.name_of(p.MOTION_STATES, state)})'
        ok, text = self.call(p.MOTION_STATE, p.motion_state_items(p.MOTION_RELEASE_SOFT_ESTOP),
                             'release sent', seq)
        if not ok:
            return False, text
        left = self.wait_until(lambda: self.motion_state() != p.MOTION_SOFT_ESTOP,
                               self._transition_timeout, seq)
        if left == 'aborted':
            return False, 'aborted by emergency_stop'
        if not left:
            return False, f'still in soft emergency stop after {self._transition_timeout:.0f} s'
        return True, (f'soft emergency stop released '
                      f'({p.name_of(p.MOTION_STATES, self.motion_state())}); cmd_vel stays held '
                      'until stand_up or rl_control')

    def motion_handler(self, name: str, state: int):
        """Return the handler of sit_down, idle or damping: confirmed by the reported state."""
        def handler(seq):
            if self.walking():
                return False, 'the robot is walking'
            current = self.motion_state()
            if state != p.MOTION_SIT and current not in SITTING_STATES:
                return False, (f'{name} only from sit, idle, soft emergency stop or damping: '
                               f'sit down first ({p.name_of(p.MOTION_STATES, current)})')
            self._gate.hold(name, time.monotonic())
            if current == state:
                return True, f'already {p.name_of(p.MOTION_STATES, state)}'
            return self.go_to(state, self._transition_timeout, seq)
        return handler

    def gait_handler(self, name: str, gait: int):
        """Return the handler of a gait switch (RL control, regular mode, both batteries)."""
        def handler(seq):
            basic = self.latest_basic()
            if self.motion_state() != p.MOTION_RL_CONTROL:
                return False, 'gaits switch in RL control only: stand_up first'
            if basic is None or basic.usage_mode != p.USAGE_REGULAR:
                return False, 'gaits switch in regular usage mode only'
            if basic.power_management != 0:
                return False, 'gaits do not switch in single battery mode'
            return self.call(p.GAIT, p.gait_items(gait), f'{name} sent', seq)
        return handler

    def command_handler(self, name: str, message, items, hold):
        """Return the handler of a plain command; hold is set before anything is sent."""
        def handler(seq):
            if hold is not None:
                self._gate.hold(hold, time.monotonic())
            return self.call(message, items, f'{name} sent', seq)
        return handler

    def regular_mode(self, seq: int):
        """Regular usage mode; lifts a navigation_mode or assist_mode hold once confirmed."""
        ok, text = self.call(p.USAGE_MODE, p.usage_mode_items(p.USAGE_REGULAR),
                             'regular_mode sent', seq)
        if ok:
            self._gate.release(NAV_HOLDS)
        return ok, text

    def query_sleep(self, seq: int):
        """Read the robot's sleep settings (1101/7); None when it does not answer."""
        self._sleep_event.clear()
        self._sleep_settings = None
        self._link.send(p.GET_SLEEP)
        self.wait_until(self._sleep_event.is_set, self._response_timeout, seq)
        return self._sleep_settings

    def set_sleep(self, sleep: bool, seq: int):
        """Enter (a hold) or leave sleep mode, keeping the robot's Auto and Time settings."""
        if sleep:
            self._gate.hold('sleep', time.monotonic())
        settings = self.query_sleep(seq)
        if self.aborted(seq):
            return False, 'aborted by emergency_stop'
        auto, minutes = settings or self._sleep_defaults
        ok, text = self.call(p.SET_SLEEP, p.sleep_items(sleep, auto, minutes),
                             'sleep sent' if sleep else 'wake sent', seq)
        if ok and settings is None:
            text += ' (settings not reported: sleep_auto and sleep_minutes used)'
        return ok, text

    def sync_time(self, seq: int):
        """Set this computer's local time on every host of the robot."""
        items = p.time_sync_items()
        return self.call(p.TIME_SYNC, items, f'time set to {items["Time"]}', seq)

    # ------------------------------------------------------------------ publishing

    def publish_reports(self):
        """Publish odometry, IMU and joints per motion report, battery and GPS per device one."""
        with self._data_lock:
            motion, new_motion = self._motion, self._new_motion
            device, new_device = self._device, self._new_device
            self._new_motion = self._new_device = False
            pose = (self._odometry.x, self._odometry.y, self._odometry.yaw)
            wheels = list(self._wheels.angles)
            twist = self._twist
            unit = self._angles.unit
        stamp = self.get_clock().now().to_msg()
        if motion is not None and new_motion:
            self._odom_pub.publish(self.odometry(twist, pose, stamp))
            self._imu_pub.publish(self.imu(motion, unit, stamp))
            if motion.joints is not None:
                self._joint_pub.publish(self.joint_state(motion, wheels, stamp))
        if device is not None and new_device:
            battery = self.battery(device, stamp)
            if battery is not None:
                self._battery_pub.publish(battery)
            if device.gps is not None:
                self._gps_pub.publish(self.gps(device.gps, stamp))

    def odometry(self, twist, pose, stamp) -> Odometry:
        """Dead-reckoned pose; twist in base_link as reported."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._v['odom_frame']
        msg.child_frame_id = self._v['base_frame']
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        q = quaternion_from_rpy(0.0, 0.0, pose[2])
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = float(twist[0])
        msg.twist.twist.linear.y = float(twist[1])
        msg.twist.twist.angular.z = float(twist[2])
        for i, variance in enumerate((0.05, 0.05, 1e6, 1e6, 1e6, 0.05)):
            msg.pose.covariance[i * 7] = variance
        for i, variance in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.05)):
            msg.twist.covariance[i * 7] = variance
        if self._tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self._v['base_frame']
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.rotation = msg.pose.pose.orientation
            self._tf.sendTransform(t)
        return msg

    def imu(self, motion, unit, stamp) -> Imu:
        """Attitude (unknown until the angle unit is), rates and acceleration as reported."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._v['imu_frame']
        if unit is not None:
            rad = self._angles.radians
            q = quaternion_from_rpy(rad(motion.roll), rad(motion.pitch), rad(motion.yaw))
            o = msg.orientation
            o.x, o.y, o.z, o.w = q
            msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.01
            msg.orientation_covariance[8] = 0.05
        else:
            msg.orientation_covariance[0] = -1.0       # unit not known yet: no orientation
        av = msg.angular_velocity
        av.x = finite_or(motion.omega_x, 0.0)
        av.y = finite_or(motion.omega_y, 0.0)
        av.z = finite_or(motion.omega_z, 0.0)
        unknown_xy = motion.omega_x is None or motion.omega_y is None
        msg.angular_velocity_covariance[0] = msg.angular_velocity_covariance[4] = (
            1e6 if unknown_xy else 0.01)
        msg.angular_velocity_covariance[8] = 0.01
        if motion.acc is not None:
            la = msg.linear_acceleration
            la.x, la.y, la.z = (finite_or(v, 0.0) for v in motion.acc)
            for i in (0, 4, 8):
                msg.linear_acceleration_covariance[i] = 0.1
        else:
            msg.linear_acceleration_covariance[0] = -1.0  # not reported
        return msg

    def joint_state(self, motion, wheels, stamp) -> JointState:
        """Leg angles as reported (no leg velocities: NaN); wheels integrated from their speed."""
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(p.JOINT_NAMES)
        positions, velocities = [], []
        wheel = 0
        for i, v in enumerate(motion.joints):
            if i in p.WHEEL_INDICES:
                positions.append(float(wheels[wheel]))
                velocities.append(finite_or(v, 0.0))
                wheel += 1
            else:
                positions.append(finite_or(v, 0.0))
                velocities.append(float('nan'))
        msg.position = positions
        msg.velocity = velocities
        return msg

    def battery(self, device, stamp):
        """Both batteries as one BatteryState; None while no battery reports in."""
        percentage, voltage, temperature, charging = battery_summary(device.batteries)
        if percentage is None and voltage is None:
            return None
        msg = BatteryState()
        msg.header.stamp = stamp
        msg.percentage = float(percentage) if percentage is not None else float('nan')
        msg.voltage = float(voltage) if voltage is not None else float('nan')
        msg.temperature = float(temperature) if temperature is not None else float('nan')
        msg.current = float('nan')
        msg.charge = float('nan')
        msg.capacity = float('nan')
        msg.design_capacity = float('nan')
        msg.present = True
        if charging is None:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        elif charging:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        else:
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.location = ', '.join(b.name for b in device.batteries)
        msg.serial_number = ', '.join(b.serial for b in device.batteries if b.serial)
        return msg

    def gps(self, gps, stamp) -> NavSatFix:
        """Return the GPS module's fix; covariance approximated from the DOP values."""
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self._v['gps_frame']
        msg.status.status = gps_status(gps.fix_quality)
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = float(gps.latitude)
        msg.longitude = float(gps.longitude)
        msg.altitude = float(gps.altitude)
        if gps.hdop > 0.0:
            horizontal = (gps.hdop * GPS_UERE) ** 2
            vertical = ((gps.vdop if gps.vdop > 0.0 else 2.0 * gps.hdop) * GPS_UERE) ** 2
            msg.position_covariance[0] = msg.position_covariance[4] = horizontal
            msg.position_covariance[8] = vertical
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        else:
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        return msg

    def state_values(self) -> dict:
        """Return the readable robot state as key/values."""
        age = self.report_age()
        with self._data_lock:
            basic, motion, device, errors = self._basic, self._motion, self._device, self._errors
            basic_time, direction = self._basic_time, self._direction
        velocity, _ = self._gate.status()
        axis = self._gate.last_axis
        values = {
            'connected': str(age <= self._state_timeout).lower(),
            'age_sec': f'{min(age, 1e6):.1f}',
            'motion_state': p.name_of(p.MOTION_STATES, self.motion_state()),
            'velocity': velocity,
            'angle_unit': self._angles.name,
            'errors': str(len(errors)),
            'cmd_axis': ' '.join(f'{axis.get(k, 0.0):.2f}' for k in ('X', 'Y', 'Yaw')),
            'heartbeat': 'paused' if self._link.heartbeats_paused else 'running',
        }
        if self._axis_refusal is not None:
            response, stamp = self._axis_refusal
            values['axis_refusal'] = (f'{response.describe()} '
                                      f'({time.monotonic() - stamp:.0f} s ago)')
        if basic is not None:
            values.update({
                'gait': p.name_of(p.GAITS, self.gait()),
                'usage_mode': p.name_of(p.USAGE_MODES, basic.usage_mode),
                'charge': p.name_of(p.CHARGE_STATES, basic.charge),
                'hard_estop': p.name_of(p.HES_STATES, basic.hes),
                'direction': p.name_of(p.DIRECTIONS, direction),
                'obstacle_avoidance': p.name_of(p.OOA_STATES, basic.ooa),
                'power_management': p.name_of(p.POWER_MODES, basic.power_management),
                'sleep': p.name_of(p.SLEEP_STATES, basic.sleep),
                'version': basic.version or 'unknown',
                'basic_age_sec': f'{time.monotonic() - basic_time:.1f}',
            })
        if motion is not None:
            values.update({
                'height_m': f'{finite_or(motion.height, float("nan")):.3f}',
                'remain_km': f'{finite_or(motion.remain_mile, float("nan")):.1f}',
            })
        if device is not None:
            percentage = battery_summary(device.batteries)[0]
            if percentage is not None:
                values['battery'] = f'{percentage:.2f}'
            if device.front_light is not None:
                values['lights'] = f'front {device.front_light}, back {device.back_light}'
        return values

    def state_level(self, values: dict):
        """Return (level, message) of the state."""
        basic = self.latest_basic()
        if values['connected'] != 'true':
            return DiagnosticStatus.ERROR, 'no reports from the robot'
        message = values['motion_state']
        if basic is not None:
            message += f', {values["gait"]}, {values["usage_mode"]} mode'
            if basic.hes == 1:
                return DiagnosticStatus.ERROR, message + ', hard emergency stop'
        if self.motion_state() == p.MOTION_SOFT_ESTOP:
            return DiagnosticStatus.ERROR, message + f', {values["velocity"]}'
        warnings = []
        velocity, warn = self._gate.status()
        if velocity == 'axis_owner_other':
            warnings.append('axis_owner_other (E006: another client holds the axis channel)')
        elif warn:
            warnings.append(velocity)
        if values['errors'] != '0':
            warnings.append(f'{values["errors"]} errors')
        battery = values.get('battery')
        if battery is not None and float(battery) < self._low_battery:
            warnings.append('low battery')
        if basic is not None and basic.sleep:
            warnings.append('sleeping')
        if self._link.heartbeats_paused:
            warnings.append('heartbeats paused')
        if warnings:
            return DiagnosticStatus.WARN, message + ', ' + ', '.join(warnings)
        return DiagnosticStatus.OK, message

    def publish_state(self):
        """Publish the readable robot state."""
        values = self.state_values()
        status = DiagnosticStatus(name='m20', hardware_id='m20')
        status.level, status.message = self.state_level(values)
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        self._state_pub.publish(status)

    def publish_invalid(self):
        """Publish the parameter errors as the state (the driver did not start)."""
        status = DiagnosticStatus(name='m20', hardware_id='m20', level=DiagnosticStatus.ERROR)
        status.message = 'invalid parameters: ' + '; '.join(self.invalid)
        status.values = [KeyValue(key='connected', value='false')]
        self._state_pub.publish(status)

    def publish_diagnostics(self):
        """Publish link health, robot errors, temperatures and power for the dashboard."""
        values = self.state_values()
        level, message = self.state_level(values)
        driver = DiagnosticStatus(name='m20_driver', hardware_id='m20', level=level,
                                  message=message)
        link = {
            'heartbeats': self._link.heartbeats,
            'rx': self._link.rx_count, 'tx': self._link.tx_count,
            'tx_errors': self._link.tx_errors, 'bad_frames': self._link.bad_frames,
            'last_refusal': (self._link.last_failure.describe()
                             if self._link.last_failure is not None else '-'),
            'full_scale_in_use': ' '.join(f'{v:.2f}' for v in self._gate.full_scale()),
        }
        for gait, estimates in self._estimator.estimates.items():
            for key, estimate in estimates.items():
                link[f'full_scale.{gait}.{key}_estimate'] = (
                    '-' if estimate is None else f'{estimate:.2f}')
        if self._sign_warning:
            driver.level = DiagnosticStatus.WARN
            driver.message += ', reported speed opposite to cmd_vel with Direction=Back'
        driver.values = [KeyValue(key=k, value=str(v)) for k, v in
                         list(values.items()) + list(link.items())]

        with self._data_lock:
            errors, device = self._errors, self._device
        faults = DiagnosticStatus(name='m20 errors', hardware_id='m20')
        if errors:
            faults.level = DiagnosticStatus.WARN
            faults.message = '; '.join(e.describe() for e in errors)
        else:
            faults.level, faults.message = DiagnosticStatus.OK, 'none'
        faults.values = [KeyValue(key=f'0x{e.code:04X}', value=e.describe()) for e in errors]

        statuses = [driver, faults]
        if device is not None:
            statuses.append(self.temperature_status(device))
            statuses.append(self.power_status(device))
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = statuses
        self._diag_pub.publish(msg)

    @staticmethod
    def temperature_status(device) -> DiagnosticStatus:
        """Report the hottest motor and driver, and each CPU."""
        status = DiagnosticStatus(name='m20 temperatures', hardware_id='m20',
                                  level=DiagnosticStatus.OK)
        values, parts = {}, []
        for kind, temps in (('motor', device.motor_temperatures),
                            ('driver', device.driver_temperatures)):
            if temps:
                hottest = max(range(len(temps)), key=lambda i: temps[i])
                values[f'{kind}_max'] = f'{temps[hottest]:.1f} ({p.JOINT_ORDER[hottest]})'
                parts.append(f'{kind} {temps[hottest]:.0f} C')
        for host, temp, little, big in device.cpu:
            values[f'cpu_{host.lower()}'] = f'{temp:.1f} C, A55 {little:.0f} %, A76 {big:.0f} %'
        status.message = ', '.join(parts) or 'no temperatures'
        status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
        return status

    def power_status(self, device) -> DiagnosticStatus:
        """Report each battery."""
        status = DiagnosticStatus(name='m20 power', hardware_id='m20', level=DiagnosticStatus.OK)
        percentage = battery_summary(device.batteries)[0]
        status.message = 'no battery data' if percentage is None else f'{percentage:.0%}'
        if percentage is not None and percentage < self._low_battery:
            status.level = DiagnosticStatus.WARN
        status.values = [
            KeyValue(key=b.name, value=(
                f'{finite_or(b.level, float("nan")):.0f} %, '
                f'{finite_or(b.voltage, float("nan")):.1f} V, '
                f'{finite_or(b.temperature, float("nan")):.1f} C'
                + (', charging' if b.charging else '')))
            for b in device.batteries
        ]
        return status

    def destroy_node(self):
        """Send zeros when axis commands went out within the last second; close the link."""
        if self._link is not None:
            try:
                if self._gate.sent_within(time.monotonic(), 1.0):
                    with self._axis_lock:
                        for _ in range(SHUTDOWN_ZEROS):
                            self._link.send(p.AXIS, p.axis_items())
                            time.sleep(0.02)
            finally:
                self._link.stop()
        super().destroy_node()


def main(args=None):
    """Spin the driver; e-stop, services, velocity and publishing on their own threads."""
    rclpy.init(args=args)
    node = M20DriverNode()
    executor = MultiThreadedExecutor(num_threads=EXECUTOR_THREADS)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: B902
        if rclpy.ok():
            raise
    finally:
        executor.shutdown()
        invalid = bool(node.invalid)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass
    if invalid:
        sys.exit(CONFIG_ERROR_EXIT)


if __name__ == '__main__':
    main()
