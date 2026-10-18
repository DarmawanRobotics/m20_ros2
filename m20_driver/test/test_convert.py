"""Attitude, odometry, wheel angles, battery and GPS conversions."""

import math

from m20_driver import protocol as p
from m20_driver.convert import (
    AngleUnitDetector,
    battery_summary,
    gps_status,
    Odometry2D,
    quaternion_from_rpy,
    WheelAngles,
)
import pytest


def test_quaternion():
    """Identity, and a pure yaw."""
    assert quaternion_from_rpy(0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0, 1.0))
    x, y, z, w = quaternion_from_rpy(0.0, 0.0, math.pi / 2)
    assert (x, y) == pytest.approx((0.0, 0.0)) and z == pytest.approx(w)


def turn(detector, yaw0, step, omega, samples, dt=0.1, t0=0.0):
    """Feed a steady turn: yaw grows by step per report; return the first decision and its time."""
    for i in range(samples):
        found = detector.observe(t0 + i * dt, yaw0 + i * step, omega)
        if found:
            return found, i * dt
    return None, None


def test_angle_unit_configured():
    """Configured rad and deg are used as given; auto knows nothing before a turn."""
    assert AngleUnitDetector('rad').radians(1.0) == 1.0
    assert AngleUnitDetector('deg').radians(180.0) == pytest.approx(math.pi)
    assert AngleUnitDetector('deg').name == 'deg'
    auto = AngleUnitDetector()
    assert auto.unit is None and auto.radians(1.0) is None and auto.name == 'unknown (auto)'
    with pytest.raises(ValueError):
        AngleUnitDetector('grad')


def test_angle_unit_detects_degrees_with_small_angles():
    """Yaw 2.86 (degrees) turning at 0.5 rad/s: 57.3 times OmegaZ, so degrees, then latched."""
    detector = AngleUnitDetector()
    unit, when = turn(detector, 2.86, math.degrees(0.05), 0.5, 60)
    assert unit == 'deg' and when < 3.0
    assert detector.name == 'deg (auto)' and detector.radians(90.0) == pytest.approx(math.pi / 2)
    assert detector.observe(10.0, 0.0, 0.5) is None               # kept for the run


def test_angle_unit_detects_radians_with_an_unwrapped_yaw():
    """Yaw 7.0 rad (more than a turn) is no reason for degrees: the rate decides, radians."""
    detector = AngleUnitDetector()
    assert turn(detector, 7.0, 0.05, 0.5, 60)[0] == 'rad'
    assert detector.radians(7.0) == 7.0


def test_angle_unit_handles_wraps_and_ignores_slow_turns():
    """Degrees wrapping at +-180 still count; turns at 0.2 rad/s decide nothing."""
    detector = AngleUnitDetector()
    step = math.degrees(-0.06)
    assert turn(detector, -170.0, step, -0.6, 60)[0] == 'deg'      # passes -180 -> 180
    slow = AngleUnitDetector()
    assert turn(slow, 0.0, math.degrees(0.02), 0.2, 200)[0] is None
    noisy = AngleUnitDetector()
    assert turn(noisy, 0.0, 0.05, -0.5, 60)[0] is None             # rate against Yaw: no vote


def test_odometry_integrates_in_the_heading():
    """Body velocities turned by the reported heading; gaps are skipped."""
    odom = Odometry2D(max_step_sec=0.5)
    assert not odom.update(0.0, 1.0, 0.0, heading=0.0)    # first sample
    assert odom.update(0.1, 1.0, 0.0, heading=0.0)
    assert (odom.x, odom.y) == pytest.approx((0.1, 0.0))
    assert odom.update(0.2, 1.0, 0.0, heading=math.pi / 2)   # facing +y now
    assert (odom.x, odom.y) == pytest.approx((0.1, 0.1))
    assert odom.update(0.3, 0.0, 1.0, heading=math.pi / 2)   # lateral +y in the body = -x
    assert (odom.x, odom.y) == pytest.approx((0.0, 0.1))
    assert not odom.update(5.0, 1.0, 0.0, heading=0.0)       # gap
    assert (odom.x, odom.y) == pytest.approx((0.0, 0.1)) and odom.yaw == 0.0
    odom.reset()
    assert (odom.x, odom.y, odom.yaw) == (0.0, 0.0, 0.0)


def test_odometry_integrates_omega_until_the_heading_is_known():
    """Without a heading the rate is integrated; the reported heading then continues from there."""
    odom = Odometry2D()
    odom.update(0.0, 0.0, 0.0, rate=1.0)
    for i in range(1, 6):
        odom.update(i * 0.1, 0.0, 0.0, rate=1.0)
    assert odom.yaw == pytest.approx(0.5)
    odom.update(0.6, 0.0, 0.0, heading=2.0)                 # unit found: no jump
    assert odom.yaw == pytest.approx(0.5)
    odom.update(0.7, 0.0, 0.0, heading=2.1)
    assert odom.yaw == pytest.approx(0.6)


def test_wheel_angles():
    """Speed * dt, wrapped to [-pi, pi)."""
    wheels = WheelAngles()
    wheels.update(0.0, [1.0, -1.0, 0.0, 10.0])
    wheels.update(0.5, [1.0, -1.0, 0.0, 10.0])
    assert wheels.angles[:3] == pytest.approx([0.5, -0.5, 0.0])
    assert -math.pi <= wheels.angles[3] < math.pi
    assert wheels.angles[3] == pytest.approx((5.0 + math.pi) % (2 * math.pi) - math.pi)


def test_battery_summary():
    """Mean level and voltage over present batteries, hottest temperature, any charging."""
    batteries = (
        p.Battery('rear', voltage=50.0, level=80.0, temperature=30.0, charging=False),
        p.Battery('front', voltage=52.0, level=60.0, temperature=35.0, charging=True),
        p.Battery('missing', voltage=0.0, level=0.0),
    )
    assert battery_summary(batteries) == (pytest.approx(0.7), 51.0, 35.0, True)
    assert battery_summary(()) == (None, None, None, None)


def test_gps_status():
    """NMEA fix quality to NavSatStatus: 1, 3 fix; 2 SBAS; 4, 5 GBAS (RTK); others no fix."""
    assert [gps_status(q) for q in range(9)] == [-1, 0, 1, 0, 2, 2, -1, -1, -1]
