#!/usr/bin/env python3
"""
ROS 2 driver of the Deep Robotics Lynx M20 over its inspection protocol (UDP, JSON).

cmd_vel becomes axis commands in regular usage mode; the robot's reports become odometry, IMU,
joint states, battery, GPS and a readable state. Every robot command is a std_srvs/Trigger
service that waits for the robot's call status response.
"""

import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from m20_driver import protocol as p
from m20_driver.convert import (
    AngleUnit,
    battery_summary,
    finite_or,
    gps_status,
    Odometry2D,
    quaternion_from_rpy,
    WheelAngles,
)
from m20_driver.link import RobotLink
from m20_driver.motion import SpeedEstimator, VelocityConfig, VelocityPolicy
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, JointState, NavSatFix, NavSatStatus
from std_srvs.srv import Trigger

try:  # Galactic and later
    from rclpy.executors import ExternalShutdownException
except ImportError:  # Foxy
    class ExternalShutdownException(Exception):
        """Placeholder: Foxy has no ExternalShutdownException."""

GPS_UERE = 4.0      # m, user range error that turns DOP into a rough standard deviation

# Plain commands: service name -> (message, items). Motion states are in MOTION_SERVICES.
COMMANDS = {
    'gait_basic': (p.GAIT, p.gait_items(p.GAIT_BASIC)),
    'gait_stair': (p.GAIT, p.gait_items(p.GAIT_STAIR)),
    'gait_flat_agile': (p.GAIT, p.gait_items(p.GAIT_FLAT_AGILE)),
    'gait_stair_agile': (p.GAIT, p.gait_items(p.GAIT_STAIR_AGILE)),
    'regular_mode': (p.USAGE_MODE, p.usage_mode_items(p.USAGE_REGULAR)),
    'navigation_mode': (p.USAGE_MODE, p.usage_mode_items(p.USAGE_NAVIGATION)),
    'assist_mode': (p.USAGE_MODE, p.usage_mode_items(p.USAGE_ASSIST)),
    'lights_on': (p.LIGHTS, p.lights_items(True, True)),
    'lights_off': (p.LIGHTS, p.lights_items(False, False)),
    'charge_start': (p.CHARGE, p.charge_items(p.CHARGE_START)),
    'charge_stop': (p.CHARGE, p.charge_items(p.CHARGE_STOP)),
}
# Motion state services: name -> (state, refused while walking)
MOTION_SERVICES = {
    'sit_down': (p.MOTION_SIT, True),
    'idle': (p.MOTION_IDLE, True),
    'damping': (p.MOTION_DAMPING, True),
    'rl_control': (p.MOTION_RL_CONTROL, False),
}
GAIT_PARAMS = {
    'basic': p.GAIT_BASIC,
    'stair': p.GAIT_STAIR,
    'flat_agile': p.GAIT_FLAT_AGILE,
    'stair_agile': p.GAIT_STAIR_AGILE,
}


class M20DriverNode(Node):
    """Bridges ROS 2 and the M20's inspection protocol."""

    def __init__(self, **kwargs):
        """Declare parameters, open the UDP link and create every interface."""
        super().__init__('m20_driver', **kwargs)
        declare = self.declare_parameter
        declare('robot_ip', p.ROBOT_IP)
        declare('robot_port', p.UDP_PORT)
        declare('local_ip', '0.0.0.0')
        declare('local_port', 0)
        declare('heartbeat_hz', 5.0)
        declare('cmd_rate_hz', 12.5)
        declare('cmd_timeout_sec', 0.5)
        declare('stop_burst_sec', 0.5)
        declare('auto_regular_mode', True)
        declare('mode_refresh_sec', 2.0)
        declare('max_vx', 1.0)
        declare('max_vy', 0.5)
        declare('max_wz', 1.0)
        declare('full_scale_vx', 5.0)
        declare('full_scale_vy', 2.0)
        declare('full_scale_wz', 3.0)
        declare('invert_vy', False)
        declare('invert_wz', False)
        declare('follow_direction', True)
        declare('angles_in_degrees', False)
        declare('odom_frame', 'odom')
        declare('base_frame', 'base_link')
        declare('imu_frame', 'imu_link')
        declare('gps_frame', 'base_link')
        declare('publish_tf', False)
        declare('state_hz', 2.0)
        declare('state_timeout_sec', 1.5)
        declare('response_timeout_sec', 1.0)
        declare('stand_timeout_sec', 10.0)
        declare('rl_timeout_sec', 5.0)
        declare('stand_up_gait', '')
        declare('sleep_auto', False)
        declare('sleep_minutes', 10)
        declare('low_battery', 0.2)

        value = lambda name: self.get_parameter(name).value  # noqa: E731
        self._odom_frame = str(value('odom_frame'))
        self._base_frame = str(value('base_frame'))
        self._imu_frame = str(value('imu_frame'))
        self._gps_frame = str(value('gps_frame'))
        self._state_timeout = float(value('state_timeout_sec'))
        self._response_timeout = float(value('response_timeout_sec'))
        self._stand_timeout = float(value('stand_timeout_sec'))
        self._rl_timeout = float(value('rl_timeout_sec'))
        self._low_battery = float(value('low_battery'))
        self._sleep_defaults = (bool(value('sleep_auto')), int(value('sleep_minutes')))
        gait = str(value('stand_up_gait')).strip()
        if gait and gait not in GAIT_PARAMS:
            self.get_logger().warn(f'stand_up_gait "{gait}" unknown, ignored '
                                   f'({", ".join(GAIT_PARAMS)})')
            gait = ''
        self._stand_up_gait = gait
        self._policy = VelocityPolicy(VelocityConfig(
            max_vx=float(value('max_vx')),
            max_vy=float(value('max_vy')),
            max_wz=float(value('max_wz')),
            full_scale_vx=float(value('full_scale_vx')),
            full_scale_vy=float(value('full_scale_vy')),
            full_scale_wz=float(value('full_scale_wz')),
            cmd_timeout_sec=float(value('cmd_timeout_sec')),
            stop_burst_sec=float(value('stop_burst_sec')),
            auto_regular_mode=bool(value('auto_regular_mode')),
            mode_refresh_sec=float(value('mode_refresh_sec')),
            invert_vy=bool(value('invert_vy')),
            invert_wz=bool(value('invert_wz')),
        ))
        self._follow_direction = bool(value('follow_direction'))
        self._estimator = SpeedEstimator()
        self._angles = AngleUnit(bool(value('angles_in_degrees')))
        self._odometry = Odometry2D()
        self._wheels = WheelAngles()

        self._data_lock = threading.Lock()
        self._basic = None
        self._basic_time = None
        self._motion = None
        self._motion_time = None
        self._new_motion = False
        self._device = None
        self._new_device = False
        self._errors = ()
        self._errors_time = None
        self._sleep_settings = None
        self._sleep_event = threading.Event()
        self._last_axis = p.axis_items()
        self._was_connected = False

        self._link = RobotLink(
            str(value('robot_ip')),
            int(value('robot_port')),
            str(value('local_ip')),
            int(value('local_port')),
            self.on_frame,
            heartbeat_hz=max(1.0, float(value('heartbeat_hz'))),   # the document: >= 1 Hz
        )

        self._tf = None
        if bool(value('publish_tf')):
            from tf2_ros import TransformBroadcaster
            self._tf = TransformBroadcaster(self)

        self._odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self._imu_pub = self.create_publisher(Imu, 'imu', 10)
        self._joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self._battery_pub = self.create_publisher(BatteryState, 'battery', 10)
        self._gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self._state_pub = self.create_publisher(DiagnosticStatus, 'state', 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)

        # services wait for the robot's answer: their own group, so timers keep running
        services = ReentrantCallbackGroup()
        self.create_service(Trigger, 'stand_up', self.on_stand_up, callback_group=services)
        self.create_service(Trigger, 'emergency_stop', self.on_emergency_stop,
                            callback_group=services)
        self.create_service(Trigger, 'sleep', self.sleep_callback(True), callback_group=services)
        self.create_service(Trigger, 'wake', self.sleep_callback(False), callback_group=services)
        self.create_service(Trigger, 'sync_time', self.on_sync_time, callback_group=services)
        for name, (state, guarded) in MOTION_SERVICES.items():
            self.create_service(Trigger, name, self.motion_callback(name, state, guarded),
                                callback_group=services)
        for name, (message, items) in COMMANDS.items():
            self.create_service(Trigger, name, self.command_callback(name, message, items),
                                callback_group=services)

        self.create_timer(0.5, self.watch_connection)
        self.create_timer(1.0 / float(value('cmd_rate_hz')), self.send_velocity)
        self.create_timer(0.02, self.publish_reports)
        self.create_timer(1.0 / float(value('state_hz')), self.publish_state)
        self.create_timer(1.0, self.publish_diagnostics)

        self._link.start()
        self.get_logger().info(
            f'M20 at {self._link.remote[0]}:{self._link.remote[1]}, this end '
            f'{self._link.local[0]}:{self._link.local[1]} (the robot reports to the heartbeat '
            'sender)'
        )

    # ------------------------------------------------------------------ link

    def on_frame(self, frame: p.Frame):
        """Store one frame from the robot (receive thread)."""
        now = time.monotonic()
        if frame.key == p.MOTION_STATUS:
            motion = p.decode_motion(frame.items)
            if self._angles.observe(motion.roll, motion.pitch, motion.yaw):
                self.get_logger().warn('reported angles exceed 2*pi: reading them as degrees')
            yaw = self._angles.radians(motion.yaw)
            with self._data_lock:
                self._odometry.update(now, finite_or(motion.linear_x, 0.0),
                                      finite_or(motion.linear_y, 0.0), yaw)
                if motion.joints is not None:
                    self._wheels.update(now, [motion.joints[i] for i in p.WHEEL_INDICES])
                self._motion = motion
                self._motion_time = now
                self._new_motion = True
                sent = self._last_axis
            self._estimator.update(sent, (motion.linear_x, motion.linear_y, motion.omega_z))
        elif frame.key == p.BASIC_STATUS:
            basic = p.decode_basic(frame.items)
            with self._data_lock:
                previous = self._basic
                self._basic = basic
                self._basic_time = now
            if basic.direction == 1 and (previous is None or previous.direction != 1):
                self.get_logger().warn(
                    'forward direction of the robot is Back (hard emergency stop side): '
                    + ('cmd_vel x and y are negated to stay in base_link' if self._follow_direction
                       else 'cmd_vel x and y act reversed (follow_direction is false)'))
        elif frame.key == p.DEVICE_STATUS:
            device = p.decode_device(frame.items)
            with self._data_lock:
                self._device = device
                self._new_device = True
        elif frame.key == p.ERROR_STATUS:
            errors = p.decode_errors(frame.items)
            with self._data_lock:
                changed = {e.code for e in errors} != {e.code for e in self._errors}
                self._errors = errors
                self._errors_time = now
            if changed and errors:
                self.get_logger().warn('robot errors: ' + '; '.join(e.describe() for e in errors))
        elif frame.key == p.GET_SLEEP and 'Sleep' in frame.items:
            self._sleep_settings = (
                bool(frame.items.get('Auto', self._sleep_defaults[0])),
                int(finite_or(frame.items.get('Time'), self._sleep_defaults[1])),
            )
            self._sleep_event.set()

    def report_age(self) -> float:
        """Return seconds since the last status report."""
        with self._data_lock:
            stamps = [t for t in (self._basic_time, self._motion_time) if t is not None]
        return time.monotonic() - max(stamps) if stamps else float('inf')

    def connected(self) -> bool:
        """Return True while status reports keep arriving."""
        return self.report_age() <= self._state_timeout

    def basic(self):
        """Return the latest basic status while it is fresh, else None."""
        with self._data_lock:
            basic, stamp = self._basic, self._basic_time
        if basic is None or stamp is None or time.monotonic() - stamp > 3.0 * self._state_timeout:
            return None
        return basic

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

    # ------------------------------------------------------------------ motion

    def on_cmd_vel(self, msg: Twist):
        """Take a velocity command (base_link, REP-103)."""
        self._policy.command(msg.linear.x, msg.linear.y, msg.angular.z, time.monotonic())

    def watch_connection(self):
        """Log when reports start and stop; read the sleep settings on every (re)connect."""
        connected = self.connected()
        if connected and not self._was_connected:
            self.get_logger().info('M20 reports arriving')
            self._link.send(p.GET_SLEEP)
        elif not connected and self._was_connected:
            self.get_logger().warn('M20 reports stopped')
        self._was_connected = connected

    def send_velocity(self):
        """Stream the axis commands of the velocity policy."""
        basic = self.basic()
        reverse = self._follow_direction and basic is not None and basic.direction == 1
        requests = self._policy.tick(time.monotonic(), basic.usage_mode if basic else None,
                                     reverse)
        for message, items in requests:
            self._link.send(message, items)
            if message == p.AXIS:
                with self._data_lock:
                    self._last_axis = items

    # ------------------------------------------------------------------ services

    def call(self, message, items, done: str):
        """Send a request and turn its call status into (success, message)."""
        sent, response = self._link.request(message, items, self._response_timeout)
        if not sent:
            return False, f'not sent: {self._link.last_error or "link closed"}'
        if response is None:
            note = '' if self.connected() else ', no reports from the robot'
            return True, f'{done} (no reply within {self._response_timeout:.1f} s{note})'
        if not response.ok:
            return False, f'robot refused: {response.describe()}'
        return True, done

    def wait_state(self, states, timeout: float) -> bool:
        """Wait until the reported motion state is one of states."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.motion_state() in states:
                return True
            time.sleep(0.05)
        return self.motion_state() in states

    def walking(self) -> bool:
        """Return True while cmd_vel moved the robot within the last second."""
        return self._policy.recently_moved(time.monotonic(), 1.0)

    def on_stand_up(self, _request, response):
        """Stand, then RL control (the document: switch to RL control soon after standing)."""
        if not self.connected():
            response.success, response.message = False, 'no reports from the robot'
            return response
        state = self.motion_state()
        if state == p.MOTION_RL_CONTROL:
            response.success, response.message = True, 'already in RL control'
            return response
        if state != p.MOTION_STAND:
            ok, text = self.call(p.MOTION_STATE, p.motion_state_items(p.MOTION_STAND),
                                 'stand sent')
            if not ok:
                response.success, response.message = False, text
                return response
            if not self.wait_state((p.MOTION_STAND, p.MOTION_RL_CONTROL), self._stand_timeout):
                response.success = False
                response.message = (f'not standing after {self._stand_timeout:.0f} s '
                                    f'({p.name_of(p.MOTION_STATES, self.motion_state())})')
                return response
        if self.motion_state() != p.MOTION_RL_CONTROL:
            ok, text = self.call(p.MOTION_STATE, p.motion_state_items(p.MOTION_RL_CONTROL),
                                 'RL control sent')
            if not ok:
                response.success, response.message = False, text
                return response
            if not self.wait_state((p.MOTION_RL_CONTROL,), self._rl_timeout):
                response.success = False
                response.message = (f'standing, but not in RL control after '
                                    f'{self._rl_timeout:.0f} s')
                return response
        message = 'standing in RL control'
        if self._stand_up_gait:
            ok, text = self.call(p.GAIT, p.gait_items(GAIT_PARAMS[self._stand_up_gait]),
                                 f'gait {self._stand_up_gait}')
            message += f', {text}' if ok else f', gait not set: {text}'
        response.success, response.message = True, message
        return response

    def on_emergency_stop(self, _request, response):
        """Zero the velocity and send the soft emergency stop."""
        self._policy.stop(time.monotonic())
        zero = p.axis_items()
        self._link.send(p.AXIS, zero)
        with self._data_lock:
            self._last_axis = zero
        response.success, response.message = self.call(
            p.MOTION_STATE, p.motion_state_items(p.MOTION_SOFT_ESTOP), 'soft emergency stop sent')
        return response

    def motion_callback(self, name: str, state: int, guarded: bool):
        """Return the service callback of a motion state switch."""
        def callback(_request, response):
            if guarded and self.walking():
                response.success, response.message = False, 'the robot is walking'
                return response
            if self.motion_state() == state:
                response.success = True
                response.message = f'already {p.name_of(p.MOTION_STATES, state)}'
                return response
            response.success, response.message = self.call(
                p.MOTION_STATE, p.motion_state_items(state), f'{name} sent')
            return response
        return callback

    def command_callback(self, name: str, message, items):
        """Return the service callback of a plain command."""
        def callback(_request, response):
            response.success, response.message = self.call(message, items, f'{name} sent')
            return response
        return callback

    def sleep_callback(self, sleep: bool):
        """
        Return the service callback that enters or leaves sleep mode.

        The robot takes Sleep, Auto and Time together: Auto and Time are kept as the robot
        reports them (1101/7), the parameters only fill in when it does not answer.
        """
        def callback(_request, response):
            if self._sleep_settings is None:
                self._sleep_event.clear()
                self._link.send(p.GET_SLEEP)
                self._sleep_event.wait(self._response_timeout)
            auto, minutes = self._sleep_settings or self._sleep_defaults
            response.success, response.message = self.call(
                p.SET_SLEEP, p.sleep_items(sleep, auto, minutes),
                'sleep sent' if sleep else 'wake sent')
            return response
        return callback

    def on_sync_time(self, _request, response):
        """Set this computer's local time on every host of the robot."""
        items = p.time_sync_items()
        response.success, response.message = self.call(p.TIME_SYNC, items,
                                                       f'time set to {items["Time"]}')
        return response

    # ------------------------------------------------------------------ publishing

    def publish_reports(self):
        """Publish odometry, IMU and joints per motion report, battery and GPS per device one."""
        with self._data_lock:
            motion, new_motion = self._motion, self._new_motion
            device, new_device = self._device, self._new_device
            self._new_motion = self._new_device = False
            pose = (self._odometry.x, self._odometry.y, self._odometry.yaw)
            wheels = list(self._wheels.angles)
        stamp = self.get_clock().now().to_msg()
        if motion is not None and new_motion:
            self._odom_pub.publish(self.odometry(motion, pose, stamp))
            self._imu_pub.publish(self.imu(motion, stamp))
            if motion.joints is not None:
                self._joint_pub.publish(self.joint_state(motion, wheels, stamp))
        if device is not None and new_device:
            battery = self.battery(device, stamp)
            if battery is not None:
                self._battery_pub.publish(battery)
            if device.gps is not None:
                self._gps_pub.publish(self.gps(device.gps, stamp))

    def odometry(self, motion, pose, stamp) -> Odometry:
        """Dead-reckoned pose from the reported velocities and heading; twist in base_link."""
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = float(pose[0])
        msg.pose.pose.position.y = float(pose[1])
        q = quaternion_from_rpy(0.0, 0.0, pose[2])
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        msg.twist.twist.linear.x = finite_or(motion.linear_x, 0.0)
        msg.twist.twist.linear.y = finite_or(motion.linear_y, 0.0)
        msg.twist.twist.angular.z = finite_or(motion.omega_z, 0.0)
        for i, variance in enumerate((0.05, 0.05, 1e6, 1e6, 1e6, 0.05)):
            msg.pose.covariance[i * 7] = variance
        for i, variance in enumerate((0.02, 0.02, 1e6, 1e6, 1e6, 0.05)):
            msg.twist.covariance[i * 7] = variance
        if self._tf is not None:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = self._base_frame
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.rotation = msg.pose.pose.orientation
            self._tf.sendTransform(t)
        return msg

    def imu(self, motion, stamp) -> Imu:
        """Attitude and rates as reported; missing fields get a huge covariance."""
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self._imu_frame
        rad = self._angles.radians
        q = quaternion_from_rpy(rad(motion.roll), rad(motion.pitch), rad(motion.yaw))
        o = msg.orientation
        o.x, o.y, o.z, o.w = q
        msg.orientation_covariance[0] = msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.05
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
        """Leg angles as reported; wheel angles integrated from the reported wheel speeds."""
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
        msg.header.frame_id = self._gps_frame
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
        values = {
            'connected': str(age <= self._state_timeout).lower(),
            'age_sec': f'{min(age, 1e6):.1f}',
            'motion_state': p.name_of(p.MOTION_STATES, self.motion_state()),
            'angle_unit': self._angles.name,
            'errors': str(len(errors)),
            'cmd_axis': ' '.join(f'{self._last_axis.get(k, 0.0):.2f}' for k in ('X', 'Y', 'Yaw')),
        }
        if basic is not None:
            values.update({
                'gait': p.name_of(p.GAITS, basic.gait),
                'usage_mode': p.name_of(p.USAGE_MODES, basic.usage_mode),
                'charge': p.name_of(p.CHARGE_STATES, basic.charge),
                'hard_estop': p.name_of(p.HES_STATES, basic.hes),
                'direction': p.name_of(p.DIRECTIONS, basic.direction),
                'obstacle_avoidance': p.name_of(p.OOA_STATES, basic.ooa),
                'power_management': p.name_of(p.POWER_MODES, basic.power_management),
                'sleep': p.name_of(p.SLEEP_STATES, basic.sleep),
                'version': basic.version or 'unknown',
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
        basic = self.basic()
        if values['connected'] != 'true':
            return DiagnosticStatus.ERROR, 'no reports from the robot'
        message = values['motion_state']
        if basic is not None:
            message += f', {values["gait"]}, {values["usage_mode"]} mode'
            if basic.hes == 1:
                return DiagnosticStatus.ERROR, message + ', hard emergency stop'
        if self.motion_state() == p.MOTION_SOFT_ESTOP:
            return DiagnosticStatus.ERROR, message
        warnings = []
        if values['errors'] != '0':
            warnings.append(f'{values["errors"]} errors')
        battery = values.get('battery')
        if battery is not None and float(battery) < self._low_battery:
            warnings.append('low battery')
        if basic is not None and basic.sleep:
            warnings.append('sleeping')
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
        }
        for key, estimate in self._estimator.estimates.items():
            link[f'full_scale_{key}_estimate'] = '-' if estimate is None else f'{estimate:.2f}'
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
        """Send one zero axis command and close the link."""
        try:
            self._link.send(p.AXIS, p.axis_items())
        finally:
            self._link.stop()
        super().destroy_node()


def main(args=None):
    """Spin the driver; services wait on their own executor threads."""
    rclpy.init(args=args)
    node = M20DriverNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: B902  already shut down by the SIGINT handler
            pass


if __name__ == '__main__':
    main()
