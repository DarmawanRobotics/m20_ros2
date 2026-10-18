"""Attitude, odometry, wheel angles, battery and GPS conversions."""

import math

from m20_driver import protocol as p
from m20_driver.convert import (
    AngleUnit,
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


def test_angle_unit_switches_to_degrees():
    """Radians by default; an angle above 2*pi can only be degrees."""
    unit = AngleUnit()
    assert unit.radians(1.0) == 1.0 and unit.name == 'rad'
    assert not unit.observe(0.1, -0.2, 3.1)
    assert unit.observe(1.0, 2.0, 120.0)
    assert unit.degrees and unit.switched and unit.name == 'deg'
    assert unit.radians(180.0) == pytest.approx(math.pi)
    assert not unit.observe(500.0)          # already switched
    assert AngleUnit(degrees=True).radians(90.0) == pytest.approx(math.pi / 2)
    assert unit.radians(None) == 0.0


def test_odometry_integrates_in_the_heading():
    """Body velocities turned by the reported heading; gaps are skipped."""
    odom = Odometry2D(max_step_sec=0.5)
    assert not odom.update(0.0, 1.0, 0.0, 0.0)      # first sample
    assert odom.update(0.1, 1.0, 0.0, 0.0)
    assert (odom.x, odom.y) == pytest.approx((0.1, 0.0))
    assert odom.update(0.2, 1.0, 0.0, math.pi / 2)   # facing +y now
    assert (odom.x, odom.y) == pytest.approx((0.1, 0.1))
    assert odom.update(0.3, 0.0, 1.0, math.pi / 2)   # lateral +y in the body = -x in odom
    assert (odom.x, odom.y) == pytest.approx((0.0, 0.1))
    assert not odom.update(5.0, 1.0, 0.0, 0.0)       # gap
    assert (odom.x, odom.y) == pytest.approx((0.0, 0.1)) and odom.yaw == 0.0
    odom.reset()
    assert (odom.x, odom.y, odom.yaw) == (0.0, 0.0, 0.0)


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
    """NMEA fix quality to NavSatStatus."""
    assert [gps_status(q) for q in (0, 1, 2, 4, 5, 6)] == [-1, 0, 1, 2, 2, 0]
