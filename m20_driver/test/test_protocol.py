"""The codec against the document's own examples (V1.3.0) and the official Android client."""

import json
import struct

from m20_driver import protocol as p
import pytest

# Appendix 1 sample: the heartbeat as the document's C++ client sends it
SAMPLE_BODY = (
    '\n    {\n        "PatrolDevice":{\n            "Type":100,\n              "Command":100,'
    '\n              "Time":"2023-01-01 00:00:00",\n              "Items":{\n              }'
    '\n          }\n    }\n    '
)


def sample_frame() -> bytes:
    """Return the bytes the document's Appendix 1 client puts on the wire."""
    body = SAMPLE_BODY.encode()
    head = bytearray(16)
    head[0:4] = b'\xeb\x91\xeb\x90'
    head[4] = len(body) & 0xFF
    head[5] = (len(body) >> 8) & 0xFF
    head[6] = 0x01          # message ID 1, little endian
    head[7] = 0x00
    head[8] = 0x01          # JSON
    return bytes(head) + body


def test_header_layout_matches_the_document_and_the_official_client():
    """16 bytes: tag, ASDU length LE, message ID LE, 0x01 JSON, 7 reserved zeros."""
    frame = p.encode(p.HEARTBEAT, message_id=1, now=0)
    tag, length, message_id, fmt, reserved = struct.unpack('<4sHHB7s', frame[:16])
    assert tag == bytes([0xEB, 0x91, 0xEB, 0x90])
    assert length == len(frame) - 16
    assert message_id == 1 and fmt == 0x01 and reserved == bytes(7)
    # Head.kt: HEAD_TAG + 2 bytes size LE + 2 bytes id LE + type byte + 7 reserved
    assert p.header(0x1234, 0xABCD) == bytes(
        [0xEB, 0x91, 0xEB, 0x90, 0x34, 0x12, 0xCD, 0xAB, 0x01, 0, 0, 0, 0, 0, 0, 0])


def test_appendix_1_sample_parses():
    """The document's sample heartbeat (with its whitespace) is read back."""
    frame = p.parse_frame(sample_frame())
    assert frame is not None
    assert frame.key == p.HEARTBEAT and frame.message_id == 1 and frame.items == {}
    assert frame.time == '2023-01-01 00:00:00'


def test_encode_matches_the_asdu_structure():
    """Build the PatrolDevice object: Type, Command, Time (local time) and Items."""
    frame = p.encode(p.MOTION_STATE, p.motion_state_items(p.MOTION_STAND), message_id=7)
    body = json.loads(frame[16:].decode())
    device = body['PatrolDevice']
    assert device['Type'] == 2 and device['Command'] == 22
    assert device['Items'] == {'MotionParam': 1}
    assert len(device['Time']) == len('2023-01-01 00:00:00')
    assert p.parse_frame(frame).items == {'MotionParam': 1}


@pytest.mark.parametrize('message, items, expected', [
    (p.USAGE_MODE, p.usage_mode_items(p.USAGE_NAVIGATION), (1101, 5, {'Mode': 1})),
    (p.GAIT, p.gait_items(p.GAIT_FLAT_AGILE), (2, 23, {'GaitParam': 0x3002})),
    (p.GAIT, p.gait_items(p.GAIT_STAIR), (2, 23, {'GaitParam': 0x1003})),
    (p.LIGHTS, p.lights_items(True, False), (1101, 2, {'Front': 1, 'Back': 0})),
    (p.CHARGE, p.charge_items(p.CHARGE_START), (2, 24, {'Charge': 1})),
    (p.SET_SLEEP, p.sleep_items(True, True, 5), (1101, 6, {'Sleep': True, 'Auto': True,
                                                           'Time': 5})),
    (p.MOTION_STATE, p.motion_state_items(p.MOTION_RL_CONTROL), (2, 22, {'MotionParam': 17})),
])
def test_requests_match_the_document(message, items, expected):
    """Type, Command and Items of each request as sections 1.2.2 to 1.2.9 give them."""
    frame = p.parse_frame(p.encode(message, items))
    assert (frame.kind, frame.command, frame.items) == expected


def test_axis_items_round_and_clamp_like_the_official_client():
    """X fwd, Y left, Yaw ccw; 2 decimals; [-1, 1]; Z, Roll, Pitch zero."""
    items = p.axis_items(x=0.4567, y=-1.7, yaw=0.004)
    assert items == {'X': 0.46, 'Y': -1.0, 'Z': 0.0, 'Roll': 0.0, 'Pitch': 0.0, 'Yaw': 0.0}
    assert str(p.axis_items()['Yaw']) == '0.0'          # never -0.0
    assert p.axis_items(x=float('nan'))['X'] == 0.0
    assert list(items) == ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']


def test_sleep_minutes_clamped_to_the_document_range():
    """Time is [5, 30] minutes."""
    assert p.sleep_items(False, True, 2)['Time'] == 5
    assert p.sleep_items(False, True, 99)['Time'] == 30


def test_basic_status_example():
    """1.3.1.1 JSON response, including the document's full-width colons and bare STD."""
    text = (
        '{"PatrolDevice": {"Type": 1002, "Command": 6, "Time": "2023-01-01 00:00:00", '
        '"Items": {"BasicStatus": {"MotionState": 17, "Gait": 12290, "Charge":0, '
        '"HES"：0, "ControlUsageMode"：1, "Direction"：1, "OOA"：2, '
        '"PowerManagement"：0, "Sleep"：0, "Version"：STD}}}}'
    ).encode()
    frame = p.parse_frame(p.header(len(text), 3) + text)
    assert frame.key == p.BASIC_STATUS
    basic = p.decode_basic(frame.items)
    assert basic.motion_state == p.MOTION_RL_CONTROL and basic.gait == p.GAIT_FLAT_AGILE
    assert basic.usage_mode == p.USAGE_NAVIGATION and basic.direction == 1 and basic.ooa == 2
    assert basic.version == 'STD'
    assert p.name_of(p.GAITS, basic.gait) == 'flat_agile'
    assert p.name_of(p.CHARGE_STATES, 5) == 'on_dock_not_charging'


def test_motion_status_example():
    """1.3.1.2: MotionStatus and MotorStatus with the 16-element Joint array."""
    joints = [0.1 * i for i in range(16)]
    items = {
        'MotionStatus': {'Roll': 0.01, 'Pitch': -0.02, 'Yaw': 1.5, 'OmegaZ': 0.2,
                         'LinearX': 0.5, 'LinearY': -0.1, 'Height': 0.4, 'Payload': 0.0,
                         'RemainMile': 8.0},
        'MotorStatus': dict(zip(p.JOINT_ORDER, joints), Joint=joints),
    }
    m = p.decode_motion(items)
    assert (m.roll, m.pitch, m.yaw, m.omega_z) == (0.01, -0.02, 1.5, 0.2)
    assert (m.linear_x, m.linear_y, m.height, m.remain_mile) == (0.5, -0.1, 0.4, 8.0)
    assert m.joints == tuple(joints)
    assert m.omega_x is None and m.acc is None    # document version: no full IMU


def test_motion_status_named_joints_and_newer_fields():
    """Named joint keys alone still give the 16 values; the official client's extra fields."""
    joints = [float(i) for i in range(16)]
    items = {
        'MotionStatus': {'Roll': 1.0, 'Pitch': 2.0, 'Yaw': 90.0, 'OmegaX': 0.1, 'OmegaY': 0.2,
                         'AngularZ': 0.3, 'AccX': 0.0, 'AccY': 0.1, 'AccZ': 9.8,
                         'MotionState': 1, 'Gait': 4097},
        'MotorStatus': dict(zip(p.JOINT_ORDER, joints)),
    }
    m = p.decode_motion(items)
    assert m.joints == tuple(joints)
    assert (m.omega_x, m.omega_y, m.omega_z) == (0.1, 0.2, 0.3)
    assert m.acc == (0.0, 0.1, 9.8)
    assert m.motion_state == 1 and m.gait == p.GAIT_BASIC


def test_device_status_example():
    """1.3.1.3: batteries ([0] rear, [1] front), temperatures, lights, GPS, CPU."""
    items = {
        'BatteryList': [
            {'Voltage': 50.0, 'BatteryLevel': 100.0, 'battery_temperature': 30.0,
             'charge': False, 'serial': 'xxx'},
            {'Voltage': 49.0, 'BatteryLevel': 80.0, 'battery_temperature': 32.0,
             'charge': True, 'serial': 'yyy'},
        ],
        'DeviceTemperature': {'Motor': [40.0] * 16, 'Driver': [35.0] * 16},
        'Led': {'Fill': {'Front': 1, 'Back': 0}},
        'GPS': {'Latitude': -6.2, 'Longitude': 106.8, 'Speed': 0.0, 'Course': 0.0,
                'FixQuality': 4, 'NumSatellites': 12, 'Altitude': 10.0, 'HDOP': 0.8,
                'VDOP': 1.0, 'PDOP': 1.3, 'VisibleSatellites': 20},
        'DevEnable': {'GPS': 1},
        'CPU': {'AOS': {'Temperature': 50.0, 'FrequencyInt': 10.0, 'FrequencyApp': 20.0}},
    }
    d = p.decode_device(items)
    assert [b.name for b in d.batteries] == ['rear', 'front']
    assert d.batteries[1].charging is True and d.batteries[0].serial == 'xxx'
    assert d.motor_temperatures == (40.0,) * 16 and d.front_light == 1 and d.back_light == 0
    assert d.gps.fix_quality == 4 and d.gps.satellites == 12 and d.gps.latitude == -6.2
    assert d.cpu == (('AOS', 50.0, 10.0, 20.0),)
    assert d.gps_enabled == 1


def test_device_status_battery_status_group():
    """Without BatteryList, BatteryStatus Left/Right gives the batteries."""
    items = {'BatteryStatus': {'VoltageLeft': 50.0, 'VoltageRight': 49.0,
                               'BatteryLevelLeft': 70.0, 'BatteryLevelRight': 60.0,
                               'battery_temperatureLeft': 30.0, 'battery_temperatureRight': 31.0,
                               'chargeLeft': 'false', 'chargeRight': 'true'}}
    d = p.decode_device(items)
    assert [(b.name, b.level, b.charging) for b in d.batteries] == [
        ('left', 70.0, False), ('right', 60.0, True)]


def test_error_component_bits_example():
    """1.3.2 [7]: component 0x21 names LeftFrontHipX (bit 0) and RightFrontHipY (bit 5)."""
    errors = p.decode_errors({'ErrorList': [{'errorCode': 0x8022, 'component': 0x21},
                                            {'errorCode': 0x8102, 'component': 0x2}]})
    assert errors[0].parts == ('LeftFrontHipX', 'RightFrontHipY')
    assert errors[0].describe() == (
        '0x8022 joint angle limit exceeded: LeftFrontHipX, RightFrontHipY')
    assert errors[1].parts == ('left_battery',)
    assert p.RobotError(0x8212, 0xFF).parts == ()
    assert 'CPU temperature overheat protection' in p.RobotError(0x8212, 0).describe()


def test_general_response():
    """1.5: same Type and Command as the request, ErrorCode and ErrorMessage."""
    frame = p.parse_frame(p.encode_response(p.GAIT, 41, 0xE006))
    response = p.decode_response(frame)
    assert response.message_id == 41 and response.kind == 2 and response.command == 23
    assert not response.ok
    assert response.describe() == '0xE006 mismatched request client'
    assert p.decode_response(p.parse_frame(p.encode_response(p.HEARTBEAT, 1))).ok
    # status reports are never responses
    report = p.parse_frame(p.encode_report(p.BASIC_STATUS, {'BasicStatus': {}, 'ErrorCode': 0}))
    assert p.decode_response(report) is None


def test_xml_asdu():
    """An XML ASDU (format 0x00) is read too."""
    body = (
        '<?xml version="1.0" encoding="UTF-8"?><PatrolDevice><Type>1002</Type>'
        '<Command>3</Command><Time>2023-01-01 00:00:00</Time><Items>'
        '<ErrorList><errorCode>32769</errorCode><component>1</component></ErrorList>'
        '<ErrorList><errorCode>33026</errorCode><component>2</component></ErrorList>'
        '</Items></PatrolDevice>'
    ).encode()
    frame = p.parse_frame(p.header(len(body), 9, p.ASDU_XML) + body)
    assert frame.key == p.ERROR_STATUS
    errors = p.decode_errors(frame.items)
    assert [(e.code, e.component) for e in errors] == [(0x8001, 1), (0x8102, 2)]


def test_malformed_frames():
    """Bad tag, short frame, broken ASDU and a missing Type all give None."""
    good = p.encode(p.HEARTBEAT)
    assert p.parse_frame(good[:10]) is None
    assert p.parse_frame(b'\x00' + good[1:]) is None
    assert p.parse_frame(good[:-3]) is None
    broken = b'{"PatrolDevice": {"Type": 1'
    assert p.parse_frame(p.header(len(broken), 0) + broken) is None
    no_type = b'{"PatrolDevice": {"Command": 1}}'
    assert p.parse_frame(p.header(len(no_type), 0) + no_type) is None
    with pytest.raises(ValueError):
        p.header(70000, 0)


def test_joint_names_follow_the_report_order():
    """URDF names in the MotorStatus.Joint order; LeftBack is the hind left leg."""
    assert len(p.JOINT_NAMES) == len(p.JOINT_ORDER) == 16
    assert p.JOINT_NAMES[:4] == ('fl_hipx_joint', 'fl_hipy_joint', 'fl_knee_joint',
                                 'fl_wheel_joint')
    assert p.JOINT_ORDER[8] == 'LeftBackHipX' and p.JOINT_NAMES[8] == 'hl_hipx_joint'
    assert [p.JOINT_NAMES[i] for i in p.WHEEL_INDICES] == [
        'fl_wheel_joint', 'fr_wheel_joint', 'hl_wheel_joint', 'hr_wheel_joint']
