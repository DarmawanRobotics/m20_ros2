#!/usr/bin/env python3
"""
A simulated Lynx M20 for the tests and for trying the driver without a robot.

It speaks the inspection protocol over UDP: answers every request with the general call
status response (same Type, Command and message ID), reports basic (2 Hz), motion (10 Hz),
device (2 Hz) and abnormal (2 Hz) status to whoever sends heartbeats, and reacts to motion
state, gait, usage mode, axis, light, charge, sleep and time requests like the robot would.

    python3 fake_robot.py --port 30000        # then: robot_ip:=127.0.0.1
"""

import argparse
import os
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from m20_driver import protocol as p  # noqa: E402


class FakeM20:
    """State machine of the robot side, independent of the socket (unit tests drive it)."""

    STAND_SEC = 1.0
    RL_SEC = 0.5
    FULL_SCALE = {'X': 2.0, 'Y': 1.0, 'Yaw': 1.5}   # m/s, m/s, rad/s at axis value 1

    def __init__(self, motion_state=p.MOTION_SIT, angles_in_degrees=False, extended=True):
        """Start sitting in regular mode; extended adds the newer firmware's IMU fields."""
        self.motion_state = motion_state
        self.gait = p.GAIT_BASIC
        self.usage_mode = p.USAGE_REGULAR
        self.charge = 0
        self.hes = 0
        self.sleep = 0
        self.sleep_auto = True
        self.sleep_minutes = 10
        self.front_light = 0
        self.back_light = 0
        self.direction = 0               # forward direction: 0 front, 1 back (hard e-stop side)
        self.errors = [(0x8001, 0x21)]
        self.angles_in_degrees = angles_in_degrees
        self.extended = extended
        self.yaw = 0.3
        self.axis = dict.fromkeys(p.AXIS_KEYS, 0.0)
        self.axis_time = None
        self.requests = []               # (Type, Command, Items) in arrival order
        self.reject = {}                 # (Type, Command) -> error code to answer with
        self.synced_time = None
        self._pending = None             # (time, motion_state)
        self.now = 0.0

    # ------------------------------------------------------------ requests

    def handle(self, frame: p.Frame, now: float):
        """Apply one request; return the error code of the call status response."""
        self.advance(now)
        self.requests.append((frame.kind, frame.command, frame.items))
        if frame.key in self.reject:
            return self.reject[frame.key]
        items = frame.items
        if frame.key == p.MOTION_STATE:
            target = int(items.get('MotionParam', -1))
            standable = (p.MOTION_SIT, p.MOTION_IDLE, p.MOTION_DAMPING)
            if target == p.MOTION_STAND and self.motion_state in standable:
                self._pending = (now + self.STAND_SEC, p.MOTION_STAND)
            elif target == p.MOTION_RL_CONTROL and self.motion_state == p.MOTION_STAND:
                self._pending = (now + self.RL_SEC, p.MOTION_RL_CONTROL)
            elif target in p.MOTION_STATES:
                self.motion_state = target
                self._pending = None
            else:
                return 0xE005
        elif frame.key == p.GAIT:
            self.gait = int(items.get('GaitParam', self.gait))
        elif frame.key == p.USAGE_MODE:
            self.usage_mode = int(items.get('Mode', self.usage_mode))
        elif frame.key == p.AXIS:
            if self.usage_mode != p.USAGE_REGULAR:
                return 0xE008
            self.axis = {k: float(items.get(k, 0.0)) for k in p.AXIS_KEYS}
            self.axis_time = now
        elif frame.key == p.LIGHTS:
            self.front_light = int(items.get('Front', self.front_light))
            self.back_light = int(items.get('Back', self.back_light))
        elif frame.key == p.CHARGE:
            self.charge = {0: 3, 1: 1, 2: 0}.get(int(items.get('Charge', 0)), self.charge)
        elif frame.key == p.SET_SLEEP:
            self.sleep = 1 if items.get('Sleep') else 0
            self.sleep_auto = bool(items.get('Auto', self.sleep_auto))
            self.sleep_minutes = int(items.get('Time', self.sleep_minutes))
        elif frame.key == p.TIME_SYNC:
            self.synced_time = items.get('Time')
        return 0

    def advance(self, now: float):
        """Finish a transition whose time has come; axis commands time out after 0.5 s."""
        self.now = now
        if self._pending is not None and now >= self._pending[0]:
            self.motion_state = self._pending[1]
            self._pending = None
        if self.axis_time is not None and now - self.axis_time > 0.5:
            self.axis = dict.fromkeys(p.AXIS_KEYS, 0.0)

    def requests_of(self, message) -> list:
        """Return the Items of every received request of one (Type, Command)."""
        return [items for kind, command, items in self.requests if (kind, command) == message]

    # ------------------------------------------------------------ reports

    def moving(self) -> bool:
        """Return True while axis commands move the robot (RL control only)."""
        return self.motion_state == p.MOTION_RL_CONTROL

    def basic_items(self) -> dict:
        """Return the Items of the basic status report."""
        return {'BasicStatus': {
            'MotionState': self.motion_state, 'Gait': self.gait, 'Charge': self.charge,
            'HES': self.hes, 'ControlUsageMode': self.usage_mode, 'Direction': self.direction,
            'OOA': 0,
            'PowerManagement': 0, 'Sleep': self.sleep, 'Version': 'STD',
        }}

    def motion_items(self) -> dict:
        """Return the Items of the motion control status report."""
        vx = self.axis['X'] * self.FULL_SCALE['X'] if self.moving() else 0.0
        vy = self.axis['Y'] * self.FULL_SCALE['Y'] if self.moving() else 0.0
        wz = self.axis['Yaw'] * self.FULL_SCALE['Yaw'] if self.moving() else 0.0
        scale = 57.29577951308232 if self.angles_in_degrees else 1.0
        status = {
            'Roll': 0.01 * scale, 'Pitch': -0.02 * scale, 'Yaw': self.yaw * scale,
            'OmegaZ': wz, 'LinearX': vx, 'LinearY': vy, 'Height': 0.42, 'Payload': 0.0,
            'RemainMile': 7.5,
        }
        if self.extended:
            status.update({'OmegaX': 0.001, 'OmegaY': -0.002, 'AccX': 0.1, 'AccY': 0.0,
                           'AccZ': 9.81, 'MotionState': self.motion_state, 'Gait': self.gait})
        joints = [0.0, 0.7, -1.4, 3.0 * vx, 0.0, 0.7, -1.4, 3.0 * vx,
                  0.0, -0.7, 1.4, 3.0 * vx, 0.0, -0.7, 1.4, 3.0 * vx]
        motor = {'Joint': joints}
        motor.update(dict(zip(p.JOINT_ORDER, joints)))
        return {'MotionStatus': status, 'MotorStatus': motor}

    def device_items(self) -> dict:
        """Return the Items of the device state report."""
        return {
            'BatteryList': [
                {'Voltage': 50.1, 'BatteryLevel': 80, 'battery_temperature': 31.0,
                 'charge': self.charge == 2, 'serial': 'REAR01'},
                {'Voltage': 50.3, 'BatteryLevel': 90, 'battery_temperature': 33.5,
                 'charge': self.charge == 2, 'serial': 'FRONT01'},
            ],
            'DeviceTemperature': {'Motor': [40.0 + i for i in range(16)],
                                  'Driver': [35.0 + i for i in range(16)]},
            'Led': {'Fill': {'Front': self.front_light, 'Back': self.back_light}},
            'GPS': {'Latitude': -6.2, 'Longitude': 106.8, 'Speed': 0.0, 'Course': 0.0,
                    'FixQuality': 1, 'NumSatellites': 9, 'Altitude': 12.0, 'HDOP': 0.9,
                    'VDOP': 1.2, 'PDOP': 1.5, 'VisibleSatellites': 14},
            'DevEnable': {'FanSpeed': 100, 'LoadPower': 1, 'GPS': 1,
                          'Lidar': {'Front': 1, 'Back': 1}, 'Video': {'Front': 1, 'Back': 1}},
            'CPU': {'AOS': {'Temperature': 55.0, 'FrequencyInt': 20.0, 'FrequencyApp': 30.0},
                    'NOS': {'Temperature': 60.0, 'FrequencyInt': 25.0, 'FrequencyApp': 35.0}},
        }

    def error_items(self) -> dict:
        """Return the Items of the abnormal status report."""
        return {'ErrorList': [{'errorCode': c, 'component': part} for c, part in self.errors]}

    def sleep_items(self) -> dict:
        """Return the Items of the sleep settings reply (1101/7)."""
        return {'Sleep': self.sleep, 'Auto': self.sleep_auto, 'Time': self.sleep_minutes}


class FakeServer:
    """Serves a FakeM20 on a UDP port of the loopback interface."""

    RATES = {p.BASIC_STATUS: 2.0, p.MOTION_STATUS: 10.0, p.DEVICE_STATUS: 2.0,
             p.ERROR_STATUS: 2.0}

    def __init__(self, robot: FakeM20 = None, port: int = 0, host: str = '127.0.0.1',
                 rate_scale: float = 1.0, respond: bool = True):
        """Bind and start serving; rate_scale speeds the reports up for tests."""
        self.robot = robot or FakeM20()
        self.respond = respond
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((host, port))
        self.sock.settimeout(0.01)
        self.port = self.sock.getsockname()[1]
        self.client = None
        self.heartbeats = 0
        self.rate_scale = rate_scale
        self._lock = threading.Lock()
        self._next = dict.fromkeys(self.RATES, 0.0)
        self._id = 0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _send(self, message, items):
        """Send a frame to the client."""
        self._id = (self._id + 1) & 0xFFFF
        self.sock.sendto(p.encode_report(message, items, self._id), self.client)

    def _loop(self):
        """Answer requests and stream reports to the heartbeat sender."""
        while self._running:
            try:
                data, address = self.sock.recvfrom(65535)
            except socket.timeout:
                data = None
            except OSError:
                return
            now = time.monotonic()
            with self._lock:
                if data is not None:
                    frame = p.parse_frame(data)
                    if frame is not None:
                        if frame.key == p.HEARTBEAT:
                            self.client = address
                            self.heartbeats += 1
                        code = self.robot.handle(frame, now)
                        if self.respond:
                            self.sock.sendto(p.encode_response(frame.key, frame.message_id,
                                                               code), address)
                        if frame.key == p.GET_SLEEP:
                            self.sock.sendto(p.encode_report(p.GET_SLEEP,
                                                             self.robot.sleep_items(),
                                                             frame.message_id), address)
                self.robot.advance(now)
                if self.client is None:
                    continue
                for message, rate in self.RATES.items():
                    if now >= self._next[message]:
                        self._next[message] = now + 1.0 / (rate * self.rate_scale)
                        self._send(message, {
                            p.BASIC_STATUS: self.robot.basic_items,
                            p.MOTION_STATUS: self.robot.motion_items,
                            p.DEVICE_STATUS: self.robot.device_items,
                            p.ERROR_STATUS: self.robot.error_items,
                        }[message]())

    def close(self):
        """Stop serving."""
        self._running = False
        self._thread.join(timeout=1.0)
        self.sock.close()


def main():
    """Serve a simulated M20 until Ctrl-C."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=p.UDP_PORT)
    parser.add_argument('--degrees', action='store_true', help='report angles in degrees')
    args = parser.parse_args()
    server = FakeServer(FakeM20(angles_in_degrees=args.degrees), args.port, args.host)
    print(f'fake M20 on {args.host}:{server.port}')
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    server.close()


if __name__ == '__main__':
    main()
