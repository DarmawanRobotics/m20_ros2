"""
Codec of the Lynx M20 inspection protocol (Deep Robotics, V1.3.0).

Every datagram is one APDU: a 16-byte header (1.1.5) and one ASDU. The header is
EB 91 EB 90 | ASDU length (uint16 LE) | message ID (uint16 LE) | ASDU format (0x00 XML, 0x01 JSON)
| 7 reserved zero bytes. The ASDU is a "PatrolDevice" object with Type, Command, Time (local time,
"YYYY-MM-DD HH:MM:SS") and Items (1.1.6). This module only builds and reads frames: no ROS, no
sockets, unit-tested on its own.
"""

from dataclasses import dataclass
import json
import math
import re
import struct
import time
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree

# ---------------------------------------------------------------- transport (1.1.2, 1.1.5)

ROBOT_IP = '10.21.31.103'
UDP_PORT = 30000
TCP_PORT = 30001

HEADER_TAG = b'\xeb\x91\xeb\x90'
HEADER = struct.Struct('<4sHHB7s')
HEADER_SIZE = HEADER.size  # 16
ASDU_XML = 0x00
ASDU_JSON = 0x01
MAX_ASDU = 0xFFFF
TIME_FORMAT = '%Y-%m-%d %H:%M:%S'

# ---------------------------------------------------------------- messages: (Type, Command)

HEARTBEAT = (100, 100)            # 1.2.1, >= 1 Hz; the robot reports to the sender
USAGE_MODE = (1101, 5)            # 1.2.2, Items.Mode
MOTION_STATE = (2, 22)            # 1.2.3, Items.MotionParam
GAIT = (2, 23)                    # 1.2.4, Items.GaitParam
AXIS = (2, 21)                    # 1.2.5, Items.X/Y/Z/Roll/Pitch/Yaw in [-1, 1], regular mode
LIGHTS = (1101, 2)                # 1.2.6, Items.Front/Back
CUSTOM_LIGHTS = (1101, 14)        # 1.2.7
CHARGE = (2, 24)                  # 1.2.8, Items.Charge
SET_SLEEP = (1101, 6)             # 1.2.9, Items.Sleep/Auto/Time
GPS_TIME_SYNC = (1101, 16)        # 1.2.10
TIME_SYNC = (1101, 15)            # 1.2.11, Items.Time
BASIC_STATUS = (1002, 6)          # 1.3.1.1, 2 Hz
MOTION_STATUS = (1002, 4)         # 1.3.1.2, 10 Hz
DEVICE_STATUS = (1002, 5)         # 1.3.1.3, 2 Hz
ERROR_STATUS = (1002, 3)          # 1.3.2, 2 Hz and on change
GET_SLEEP = (1101, 7)             # 1.3.3
# M20 Pro only (1.4): not used by the driver
LOCATION_RESET = (2101, 1)
LOCATION = (1007, 2)
PERCEPTION_STATUS = (2002, 1)
NAV_TASK = (1003, 1)
NAV_CANCEL = (1004, 1)
NAV_STATE = (1007, 1)

STATUS_REPORTS = (BASIC_STATUS, MOTION_STATUS, DEVICE_STATUS, ERROR_STATUS)

# ---------------------------------------------------------------- values

USAGE_REGULAR = 0
USAGE_NAVIGATION = 1
USAGE_ASSIST = 2
USAGE_MODES = {0: 'regular', 1: 'navigation', 2: 'assist'}

MOTION_IDLE = 0
MOTION_STAND = 1
MOTION_SOFT_ESTOP = 2
MOTION_DAMPING = 3
MOTION_SIT = 4
MOTION_RL_CONTROL = 17
# From the SDK, not in V1.3.0 (RobotProtocol.motionMsg, RobotStatesExpansion, ControlActivity):
MOTION_NOT_READY = -1             # reported: the motion program is not running yet
MOTION_RELEASE_SOFT_ESTOP = -2    # request: leave the soft emergency stop (sendExitDampingMode)
MOTION_SET_ZERO = 5               # reported: joint zero calibration
MOTION_STATES = {
    -2: 'soft_emergency_stop_released',
    -1: 'motion_program_not_ready',
    0: 'idle',
    1: 'stand',
    2: 'soft_emergency_stop',
    3: 'power_on_damping',
    4: 'sit',
    5: 'set_zero',
    17: 'rl_control',
}

GAIT_BASIC = 0x1001               # Standard Motion Mode
GAIT_HIGH_OBSTACLES = 0x1002      # Standard Motion Mode (status table of 1.3.1.1)
GAIT_STAIR = 0x1003               # Standard Motion Mode (switching table of 1.2.4)
GAIT_FLAT_AGILE = 0x3002          # Agile Motion Mode
GAIT_STAIR_AGILE = 0x3003         # Agile Motion Mode
GAITS = {
    0x1001: 'basic',
    0x1002: 'high_obstacles',
    0x1003: 'stair',
    0x3002: 'flat_agile',
    0x3003: 'stair_agile',
}

CHARGE_STOP = 0                   # the SDK: end / leave charging, valid while charging (2, 5)
CHARGE_START = 1
CHARGE_CLEAR = 2                  # forcibly cuts the current and resets: not offered by the driver
CHARGE_CANCEL = 3                 # from the SDK, not in V1.3.0: cancel going to the dock
CHARGE_STOP_EXIT = 4              # from the SDK, not in V1.3.0: stop leaving the dock
CHARGE_STATES = {
    0: 'idle',
    1: 'entering_dock',
    2: 'charging',
    3: 'exiting_dock',
    4: 'error',
    5: 'on_dock_not_charging',
    0x1000: 'charge_error',       # from the SDK, not in V1.3.0
}
CHARGING_STATES = (1, 2, 3)       # docking, charging, leaving the dock: not to be driven
HES_STATES = {0: 'not_triggered', 1: 'triggered'}
DIRECTIONS = {0: 'front', 1: 'back'}
OOA_STATES = {0: 'not_started', 1: 'idle', 2: 'not_triggered', 3: 'avoiding'}
POWER_MODES = {0: 'regular', 1: 'single_battery'}
SLEEP_STATES = {0: 'awake', 1: 'sleeping', 2: 'entering_sleep'}

AXIS_KEYS = ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw')
AXIS_DECIMALS = 2                 # the official client rounds to 0.01

# MotorStatus.Joint / DeviceTemperature.Motor order (1.3.1.2 [3]) as the URDF names them.
# Left = battery compartment side, back = hard emergency stop side; LeftBack = hl.
JOINT_ORDER = (
    'LeftFrontHipX', 'LeftFrontHipY', 'LeftFrontKnee', 'LeftFrontWheel',
    'RightFrontHipX', 'RightFrontHipY', 'RightFrontKnee', 'RightFrontWheel',
    'LeftBackHipX', 'LeftBackHipY', 'LeftBackKnee', 'LeftBackWheel',
    'RightBackHipX', 'RightBackHipY', 'RightBackKnee', 'RightBackWheel',
)
JOINT_NAMES = tuple(
    f'{leg}_{part}_joint'
    for leg in ('fl', 'fr', 'hl', 'hr')
    for part in ('hipx', 'hipy', 'knee', 'wheel')
)
WHEEL_INDICES = (3, 7, 11, 15)    # their Joint value is a speed (rad/s), the others an angle

# ErrorList.errorCode (1.3.2)
ERROR_CODES = {
    0x8001: 'motor temperature warning',
    0x8002: 'motor over-temperature protection',
    0x8003: 'motor temperature critical shutdown',
    0x8007: 'joint driver over-temperature',
    0x8008: 'driver undervoltage protection',
    0x8009: 'driver overvoltage protection',
    0x8012: 'joint driver communication timeout',
    0x8016: 'no encoder value',
    0x8020: 'driver overcurrent protection',
    0x8021: 'temperature sensor disconnected',
    0x8022: 'joint angle limit exceeded',
    0x8024: 'joint data is NaN',
    0x8025: 'joint data update error',
    0x8027: 'body attitude error',
    0x8028: 'driver status error',
    0x8029: 'motion attitude error',
    0x8030: 'joint driver over-temperature warning',
    0x8102: 'low battery warning',
    0x8103: 'protected battery level',
    0x8106: 'battery output minimum voltage protection',
    0x8107: 'battery discharge over-temperature protection',
    0x8108: 'battery cell undervoltage protection',
    0x8112: 'battery discharge low-temperature protection',
    0x8115: 'battery charging over-temperature protection',
    0x8116: 'battery charge low-temperature protection',
    0x8117: 'battery cell overvoltage protection',
    0x8118: 'battery pack overvoltage protection',
    0x8119: 'battery pack undervoltage protection',
    0x8120: 'battery charging overcurrent protection',
    0x8121: 'battery discharge overcurrent protection',
    0x8122: 'short circuit protection',
    0x8123: 'battery front-end detection IC error',
    0x8124: 'battery software MOS lock',
    0x8125: 'battery discharge over-temperature warning',
    0x8126: 'battery discharge low-temperature warning',
    0x8127: 'battery charging over-temperature warning',
    0x8128: 'battery charging low-temperature warning',
    0x8129: 'battery output minimum voltage warning',
    0x8201: 'CPU usage overload warning',
    0x8202: 'CPU temperature overheat warning',
    0x8211: 'CPU usage overload protection',
    0x8212: 'CPU temperature overheat protection',
    0x8501: 'positioning data timeout during autonomous charging',
    0x8502: 'reflector localization failed to start during autonomous charging',
    0x8503: 'abnormal positioning during autonomous charging',
    0x8506: 'no current at the charging dock',
    0x8507: 'charging obstacle stop error',
    0x8509: 'dock entry timeout during autonomous charging',
    0x8510: 'dock withdrawal timeout during autonomous charging',
}
# Battery component bits (1.3.2 [7]): bit 0 the right battery, bit 1 the left one. 1.3.1.3 [4]
# puts the right battery on the hard emergency stop side, the rear: BatteryList[0] is the same one.
BATTERY_PARTS = ('rear (right) battery', 'front (left) battery')

# General protocol interface call status (1.5)
RESPONSE_CODES = {
    0x0000: 'success',
    0xE001: 'unsupported data format',
    0xE002: 'data parsing failed',
    0xE003: 'unsupported protocol type',
    0xE004: 'missing necessary fields',
    0xE005: 'mismatched data type',
    0xE006: 'mismatched request client',
    0xE007: 'no operation permission',
    0xE008: 'not allowed operation',
    0xE009: 'operation failure',
    0xE00A: 'unsupported function',
    0xE00B: 'internal error',
}


# ---------------------------------------------------------------- frames


@dataclass(frozen=True)
class Frame:
    """One received APDU."""

    message_id: int
    asdu_format: int
    kind: int          # Type
    command: int
    time: str
    items: dict

    @property
    def key(self) -> Tuple[int, int]:
        """Return (Type, Command)."""
        return (self.kind, self.command)


def local_time(now: Optional[float] = None) -> str:
    """Return the protocol's Time string for a POSIX time (local time zone)."""
    return time.strftime(TIME_FORMAT, time.localtime(time.time() if now is None else now))


def header(length: int, message_id: int, asdu_format: int = ASDU_JSON) -> bytes:
    """Return the 16-byte protocol header."""
    if not 0 <= length <= MAX_ASDU:
        raise ValueError(f'ASDU of {length} bytes (at most {MAX_ASDU})')
    return HEADER.pack(HEADER_TAG, length, message_id & 0xFFFF, asdu_format, bytes(7))


def encode(message: Tuple[int, int], items: Optional[dict] = None, message_id: int = 0,
           now: Optional[float] = None) -> bytes:
    """Return one JSON request frame."""
    asdu = {
        'PatrolDevice': {
            'Type': int(message[0]),
            'Command': int(message[1]),
            'Time': local_time(now),
            'Items': items if items is not None else {},
        }
    }
    body = json.dumps(asdu, separators=(',', ':'), ensure_ascii=False).encode('utf-8')
    return header(len(body), message_id) + body


def _lenient_json(text: str):
    """Parse JSON, tolerating the document's typos (full-width colons, bare words)."""
    try:
        return json.loads(text)
    except ValueError:
        fixed = text.replace('：', ':').replace('，', ',')
        fixed = re.sub(
            r':\s*(?!true\b|false\b|null\b)([A-Za-z_][A-Za-z0-9_]*)\s*([,}\]])', r':"\1"\2',
            fixed,
        )
        return json.loads(fixed)


def _xml_value(text: Optional[str]):
    """Return a number, a bool or the text of an XML leaf."""
    if text is None:
        return ''
    value = text.strip()
    for kind in (int, float):
        try:
            return kind(value)
        except ValueError:
            pass
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    return value


def _xml_to_dict(element) -> object:
    """Turn an XML element into nested dicts; repeated tags become lists."""
    children = list(element)
    if not children:
        return _xml_value(element.text)
    out: Dict[str, object] = {}
    for child in children:
        value = _xml_to_dict(child)
        if child.tag in out:
            if not isinstance(out[child.tag], list):
                out[child.tag] = [out[child.tag]]
            out[child.tag].append(value)
        else:
            out[child.tag] = value
    return out


def _asdu(body: bytes, asdu_format: int) -> Optional[dict]:
    """Decode the ASDU into the PatrolDevice dict, None when it cannot be read."""
    text = body.decode('utf-8', errors='replace').strip().strip('\x00')
    if not text:
        return None
    try:
        if asdu_format == ASDU_XML or text.startswith('<'):
            root = ElementTree.fromstring(text)
            data = _xml_to_dict(root)
            return data if isinstance(data, dict) else None
        data = _lenient_json(text)
    except (ValueError, ElementTree.ParseError):
        return None
    if isinstance(data, dict) and isinstance(data.get('PatrolDevice'), dict):
        return data['PatrolDevice']
    return data if isinstance(data, dict) else None


def parse_frame(data: bytes) -> Optional[Frame]:
    """Read one APDU; None when the header or the ASDU is malformed."""
    if len(data) < HEADER_SIZE:
        return None
    tag, length, message_id, asdu_format, _ = HEADER.unpack_from(data)
    if tag != HEADER_TAG or len(data) < HEADER_SIZE + length:
        return None
    asdu = _asdu(bytes(data[HEADER_SIZE:HEADER_SIZE + length]), asdu_format)
    if asdu is None:
        return None
    try:
        kind, command = int(asdu.get('Type')), int(asdu.get('Command'))
    except (TypeError, ValueError):
        return None
    items = asdu.get('Items')
    return Frame(
        message_id=message_id,
        asdu_format=asdu_format,
        kind=kind,
        command=command,
        time=str(asdu.get('Time', '')),
        items=items if isinstance(items, dict) else {},
    )


# ---------------------------------------------------------------- requests


def rounded(value: float) -> float:
    """Round an axis value like the official client: 2 decimals, clamped to [-1, 1]."""
    if not math.isfinite(value):
        return 0.0
    return round(max(-1.0, min(1.0, float(value))), AXIS_DECIMALS) + 0.0  # no -0.0


def axis_items(x: float = 0.0, y: float = 0.0, yaw: float = 0.0, z: float = 0.0,
               roll: float = 0.0, pitch: float = 0.0) -> dict:
    """
    Return the Items of an axis command.

    Ratios of the maximum speed in [-1, 1] with REP-103 signs: x forward, y left, yaw
    counter-clockwise (the official client's convention).
    """
    values = (x, y, z, roll, pitch, yaw)
    return {key: rounded(v) for key, v in zip(AXIS_KEYS, values)}


def usage_mode_items(mode: int) -> dict:
    """Return the Items of a usage mode switch."""
    return {'Mode': int(mode)}


def motion_state_items(state: int) -> dict:
    """Return the Items of a motion state switch."""
    return {'MotionParam': int(state)}


def gait_items(gait: int) -> dict:
    """Return the Items of a gait switch."""
    return {'GaitParam': int(gait)}


def lights_items(front: bool, back: bool) -> dict:
    """Return the Items of the flashlight command."""
    return {'Front': 1 if front else 0, 'Back': 1 if back else 0}


def charge_items(charge: int) -> dict:
    """Return the Items of the autonomous charging command."""
    return {'Charge': int(charge)}


def sleep_items(sleep: bool, auto: bool, minutes: int) -> dict:
    """Return the Items of the sleep settings (minutes in [5, 30])."""
    return {'Sleep': bool(sleep), 'Auto': bool(auto), 'Time': int(max(5, min(30, minutes)))}


def time_sync_items(now: Optional[float] = None) -> dict:
    """Return the Items of the external time synchronization: this computer's time."""
    return {'Time': local_time(now)}


# ---------------------------------------------------------------- reports


def _num(data: dict, key: str, kind=float, default=None):
    """Return data[key] as a number, the default when missing or unreadable."""
    try:
        value = data[key]
    except (KeyError, TypeError):
        return default
    if isinstance(value, bool):
        return kind(value)
    try:
        return kind(float(value)) if kind is int else kind(value)
    except (TypeError, ValueError):
        pass
    if kind is int and isinstance(value, str):
        try:
            return int(value.strip(), 0)          # '0xE006'
        except ValueError:
            pass
    return default


def _floats(values, count: int) -> Optional[Tuple[float, ...]]:
    """Return a tuple of count floats, None when the list is missing or short."""
    if not isinstance(values, (list, tuple)) or len(values) < count:
        return None
    try:
        return tuple(float(v) for v in values[:count])
    except (TypeError, ValueError):
        return None


def name_of(table: dict, value: Optional[int]) -> str:
    """Return the readable name of a code, 'unknown' when it is not in the table."""
    if value is None:
        return 'unknown'
    return table.get(value, f'unknown ({value})')


@dataclass(frozen=True)
class BasicStatus:
    """Basic status report (1002/6)."""

    motion_state: Optional[int] = None
    gait: Optional[int] = None
    charge: Optional[int] = None
    hes: Optional[int] = None
    usage_mode: Optional[int] = None
    direction: Optional[int] = None
    ooa: Optional[int] = None
    power_management: Optional[int] = None
    sleep: Optional[int] = None
    version: str = ''


def decode_basic(items: dict) -> BasicStatus:
    """Decode Items of the basic status report."""
    b = items.get('BasicStatus', items) if isinstance(items, dict) else {}
    if not isinstance(b, dict):
        b = {}
    return BasicStatus(
        motion_state=_num(b, 'MotionState', int),
        gait=_num(b, 'Gait', int),
        charge=_num(b, 'Charge', int),
        hes=_num(b, 'HES', int),
        usage_mode=_num(b, 'ControlUsageMode', int),
        direction=_num(b, 'Direction', int),
        ooa=_num(b, 'OOA', int),
        power_management=_num(b, 'PowerManagement', int),
        sleep=_num(b, 'Sleep', int),
        version=str(b.get('Version', '')),
    )


@dataclass(frozen=True)
class MotionStatus:
    """Motion control status report (1002/4). Angles as reported (rad in the document)."""

    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    omega_x: Optional[float] = None   # newer firmware (official client)
    omega_y: Optional[float] = None
    omega_z: Optional[float] = None
    acc: Optional[Tuple[float, float, float]] = None
    linear_x: Optional[float] = None
    linear_y: Optional[float] = None
    height: Optional[float] = None
    remain_mile: Optional[float] = None
    joints: Optional[Tuple[float, ...]] = None   # JOINT_ORDER: 12 angles + 4 wheel speeds
    gait: Optional[int] = None
    motion_state: Optional[int] = None


def decode_motion(items: dict) -> MotionStatus:
    """Decode Items of the motion control status report."""
    m = items.get('MotionStatus', {}) if isinstance(items, dict) else {}
    motor = items.get('MotorStatus', {}) if isinstance(items, dict) else {}
    m = m if isinstance(m, dict) else {}
    motor = motor if isinstance(motor, dict) else {}
    joints = _floats(motor.get('Joint', motor.get('joint')), len(JOINT_ORDER))
    if joints is None and all(name in motor for name in JOINT_ORDER):
        joints = _floats([motor[name] for name in JOINT_ORDER], len(JOINT_ORDER))
    acc = None
    if all(key in m for key in ('AccX', 'AccY', 'AccZ')):
        acc = (_num(m, 'AccX', default=0.0), _num(m, 'AccY', default=0.0),
               _num(m, 'AccZ', default=0.0))
    omega_z = _num(m, 'OmegaZ')
    if omega_z is None:
        omega_z = _num(m, 'AngularZ')
    return MotionStatus(
        roll=_num(m, 'Roll'),
        pitch=_num(m, 'Pitch'),
        yaw=_num(m, 'Yaw'),
        omega_x=_num(m, 'OmegaX'),
        omega_y=_num(m, 'OmegaY'),
        omega_z=omega_z,
        acc=acc,
        linear_x=_num(m, 'LinearX'),
        linear_y=_num(m, 'LinearY'),
        height=_num(m, 'Height'),
        remain_mile=_num(m, 'RemainMile'),
        joints=joints,
        gait=_num(m, 'Gait', int),
        motion_state=_num(m, 'MotionState', int),
    )


@dataclass(frozen=True)
class Battery:
    """One battery of the device report."""

    name: str
    voltage: Optional[float] = None
    level: Optional[float] = None         # percent, 0-100
    temperature: Optional[float] = None
    charging: Optional[bool] = None
    serial: str = ''


@dataclass(frozen=True)
class Gps:
    """GPS group of the device report."""

    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    speed_kmh: float = 0.0
    course: float = 0.0
    fix_quality: float = 0.0
    satellites: int = 0
    visible_satellites: int = 0
    hdop: float = 0.0
    vdop: float = 0.0
    pdop: float = 0.0


@dataclass(frozen=True)
class DeviceStatus:
    """Device state report (1002/5)."""

    batteries: Tuple[Battery, ...] = ()
    motor_temperatures: Optional[Tuple[float, ...]] = None    # JOINT_ORDER
    driver_temperatures: Optional[Tuple[float, ...]] = None
    front_light: Optional[int] = None
    back_light: Optional[int] = None
    gps: Optional[Gps] = None
    gps_enabled: Optional[int] = None
    cpu: Tuple[Tuple[str, float, float, float], ...] = ()     # (host, temp, A55 %, A76 %)


def _bool(value) -> Optional[bool]:
    """Return a bool from true/false/1/0, None when missing."""
    if value is None:
        return None
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1')
    return bool(value)


def _batteries(items: dict) -> Tuple[Battery, ...]:
    """Return the batteries of BatteryList ([0] rear, [1] front), else of BatteryStatus."""
    out: List[Battery] = []
    listed = items.get('BatteryList')
    if isinstance(listed, dict):
        listed = [listed]
    if isinstance(listed, list) and listed:
        for index, b in enumerate(listed):
            if not isinstance(b, dict):
                continue
            out.append(Battery(
                name=('rear', 'front')[index] if index < 2 else f'battery_{index}',
                voltage=_num(b, 'Voltage'),
                level=_num(b, 'BatteryLevel'),
                temperature=_num(b, 'battery_temperature'),
                charging=_bool(b.get('charge')),
                serial=str(b.get('serial', '')),
            ))
        return tuple(out)
    status = items.get('BatteryStatus')
    if isinstance(status, dict):
        for side in ('Left', 'Right'):
            if f'BatteryLevel{side}' not in status and f'Voltage{side}' not in status:
                continue
            out.append(Battery(
                name=side.lower(),
                voltage=_num(status, f'Voltage{side}'),
                level=_num(status, f'BatteryLevel{side}'),
                temperature=_num(status, f'battery_temperature{side}'),
                charging=_bool(status.get(f'charge{side}')),
                serial=str(status.get(f'serial{side}', '')),
            ))
    return tuple(out)


def _temperatures(temps: dict, kind: str) -> Optional[Tuple[float, ...]]:
    """
    Return the 16 Motor or Driver temperatures: the array, else the named fields.

    Older firmware sends LeftFrontHipXMotor, LeftFrontHipXDriver, ... (the SDK still reads them).
    """
    values = _floats(temps.get(kind), len(JOINT_ORDER))
    if values is None and all(name + kind in temps for name in JOINT_ORDER):
        values = _floats([temps[name + kind] for name in JOINT_ORDER], len(JOINT_ORDER))
    return values


def decode_device(items: dict) -> DeviceStatus:
    """Decode Items of the device state report."""
    if not isinstance(items, dict):
        return DeviceStatus()
    temps = items.get('DeviceTemperature', {})
    temps = temps if isinstance(temps, dict) else {}
    led = items.get('Led', {})
    fill = led.get('Fill', {}) if isinstance(led, dict) else {}
    fill = fill if isinstance(fill, dict) else {}
    gps = None
    g = items.get('GPS')
    if isinstance(g, dict):
        gps = Gps(
            latitude=_num(g, 'Latitude', default=0.0),
            longitude=_num(g, 'Longitude', default=0.0),
            altitude=_num(g, 'Altitude', default=0.0),
            speed_kmh=_num(g, 'Speed', default=0.0),
            course=_num(g, 'Course', default=0.0),
            fix_quality=_num(g, 'FixQuality', default=0.0),
            satellites=_num(g, 'NumSatellites', int, 0),
            visible_satellites=_num(g, 'VisibleSatellites', int, 0),
            hdop=_num(g, 'HDOP', default=0.0),
            vdop=_num(g, 'VDOP', default=0.0),
            pdop=_num(g, 'PDOP', default=0.0),
        )
    enable = items.get('DevEnable', {})
    enable = enable if isinstance(enable, dict) else {}
    cpu = []
    c = items.get('CPU', {})
    if isinstance(c, dict):
        for host in ('AOS', 'NOS', 'GOS'):
            h = c.get(host)
            if isinstance(h, dict):
                cpu.append((host, _num(h, 'Temperature', default=float('nan')),
                            _num(h, 'FrequencyInt', default=float('nan')),
                            _num(h, 'FrequencyApp', default=float('nan'))))
    return DeviceStatus(
        batteries=_batteries(items),
        motor_temperatures=_temperatures(temps, 'Motor'),
        driver_temperatures=_temperatures(temps, 'Driver'),
        front_light=_num(fill, 'Front', int),
        back_light=_num(fill, 'Back', int),
        gps=gps,
        gps_enabled=_num(enable, 'GPS', int),
        cpu=tuple(cpu),
    )


@dataclass(frozen=True)
class RobotError:
    """One entry of the abnormal status report (1002/3)."""

    code: int
    component: int

    @property
    def text(self) -> str:
        """Return the document's meaning of the code."""
        return ERROR_CODES.get(self.code, f'error 0x{self.code:04X}')

    @property
    def parts(self) -> Tuple[str, ...]:
        """Return the joints (or batteries) the component bits name."""
        names: Sequence[str] = BATTERY_PARTS if 0x8100 <= self.code < 0x8200 else JOINT_ORDER
        if not (0x8000 <= self.code < 0x8200):
            return ()
        return tuple(name for bit, name in enumerate(names) if self.component >> bit & 1)

    def describe(self) -> str:
        """Return '0x8001 motor temperature warning: LeftFrontHipX'."""
        where = ', '.join(self.parts)
        return f'0x{self.code:04X} {self.text}' + (f': {where}' if where else '')


def decode_errors(items: dict) -> Tuple[RobotError, ...]:
    """Decode Items of the abnormal status report."""
    listed = items.get('ErrorList', []) if isinstance(items, dict) else []
    if isinstance(listed, dict):
        listed = [listed]
    out = []
    for entry in listed if isinstance(listed, list) else []:
        if not isinstance(entry, dict):
            continue
        code = _num(entry, 'errorCode', int, _num(entry, 'ErrorCode', int))
        if code is None:
            continue
        out.append(RobotError(code=code, component=_num(entry, 'component', int, 0) or 0))
    return tuple(out)


@dataclass(frozen=True)
class Response:
    """General protocol interface call status (1.5)."""

    message_id: int
    kind: int          # Type
    command: int
    error_code: int
    error_message: str

    @property
    def ok(self) -> bool:
        """Return True for ErrorCode 0."""
        return self.error_code == 0

    def describe(self) -> str:
        """Return the robot's answer as text."""
        if self.ok:
            return self.error_message or 'success'
        text = self.error_message or RESPONSE_CODES.get(self.error_code, 'error')
        return f'0x{self.error_code:04X} {text}'


def decode_response(frame: Frame) -> Optional[Response]:
    """Return the call status carried by a frame, None for status reports."""
    if frame.key in STATUS_REPORTS or 'ErrorCode' not in frame.items:
        return None
    code = _num(frame.items, 'ErrorCode', int)
    if code is None:
        return None
    return Response(
        message_id=frame.message_id,
        kind=frame.kind,
        command=frame.command,
        error_code=code,
        error_message=str(frame.items.get('ErrorMessage', '')),
    )


# ---------------------------------------------------------------- reports as the robot sends them
# (the simulator and the tests)


def encode_report(message: Tuple[int, int], items: dict, message_id: int = 0,
                  now: Optional[float] = None) -> bytes:
    """Return a report or response frame as the robot would send it."""
    return encode(message, items, message_id, now)


def encode_response(message: Tuple[int, int], message_id: int, error_code: int = 0,
                    error_message: Optional[str] = None) -> bytes:
    """Return the general call status response to a request."""
    text = error_message if error_message is not None else (
        'Success' if error_code == 0 else RESPONSE_CODES.get(error_code, 'error'))
    return encode(message, {'ErrorCode': int(error_code), 'ErrorMessage': text}, message_id)
