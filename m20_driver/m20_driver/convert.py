"""Conversions without ROS: attitude, dead-reckoned odometry, wheel angles, battery."""

import math
from typing import Optional, Sequence, Tuple

TWO_PI = 2.0 * math.pi


def quaternion_from_rpy(roll: float, pitch: float,
                        yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w) of the ZYX Euler angles in radians."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def finite_or(value: Optional[float], default: float) -> float:
    """Return the value when it is a finite number, else the default."""
    if value is None:
        return default
    try:
        value = float(value)
    except (TypeError, ValueError):
        return default
    return value if math.isfinite(value) else default


class AngleUnit:
    """
    Radians as the document says, degrees once a reported angle cannot be radians.

    The protocol document gives Roll/Pitch/Yaw in rad, the official Android client calls them
    degrees. A magnitude above 2*pi only fits degrees: from then on they are converted.
    """

    def __init__(self, degrees: bool = False):
        """Start with the configured unit."""
        self.degrees = bool(degrees)
        self.switched = False

    def observe(self, *angles: Optional[float]) -> bool:
        """Look at reported angles; return True when this call switched to degrees."""
        if self.degrees:
            return False
        if any(a is not None and math.isfinite(a) and abs(a) > TWO_PI + 1e-3 for a in angles):
            self.degrees = True
            self.switched = True
            return True
        return False

    def radians(self, value: Optional[float]) -> float:
        """Return a reported angle as radians, 0 when missing."""
        value = finite_or(value, 0.0)
        return math.radians(value) if self.degrees else value

    @property
    def name(self) -> str:
        """Return 'deg' or 'rad'."""
        return 'deg' if self.degrees else 'rad'


class Odometry2D:
    """Planar dead reckoning from body velocities and the reported heading."""

    def __init__(self, max_step_sec: float = 0.5):
        """Start at the origin; gaps longer than max_step_sec are not integrated."""
        self.max_step_sec = max_step_sec
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last: Optional[float] = None

    def reset(self):
        """Go back to the origin."""
        self.x = self.y = self.yaw = 0.0
        self._last = None

    def update(self, now: float, vx: float, vy: float, yaw: float) -> bool:
        """
        Integrate body velocities (m/s) over the time since the last update.

        yaw (rad) is the heading the robot reports; the pose turns with it. Return False when
        the step was skipped (first sample, a gap, or time going backwards).
        """
        last, self._last = self._last, now
        self.yaw = yaw
        if last is None:
            return False
        dt = now - last
        if dt <= 0.0 or dt > self.max_step_sec:
            return False
        c, s = math.cos(yaw), math.sin(yaw)
        self.x += (c * vx - s * vy) * dt
        self.y += (s * vx + c * vy) * dt
        return True


class WheelAngles:
    """Wheel angles integrated from the reported wheel speeds (for the URDF only)."""

    def __init__(self, count: int = 4, max_step_sec: float = 0.5):
        """Start at zero."""
        self.angles = [0.0] * count
        self.max_step_sec = max_step_sec
        self._last: Optional[float] = None

    def update(self, now: float, speeds: Sequence[float]):
        """Add speed * dt to each wheel; wrap to [-pi, pi)."""
        last, self._last = self._last, now
        if last is None:
            return
        dt = now - last
        if dt <= 0.0 or dt > self.max_step_sec:
            return
        for i, speed in enumerate(speeds[:len(self.angles)]):
            angle = self.angles[i] + finite_or(speed, 0.0) * dt
            self.angles[i] = (angle + math.pi) % TWO_PI - math.pi


def battery_summary(batteries) -> Tuple[Optional[float], Optional[float], Optional[float],
                                        Optional[bool]]:
    """
    Return (percentage 0-1, voltage, max temperature, charging) over the present batteries.

    A battery counts as present when it reports a voltage or a level above zero.
    """
    present = [b for b in batteries if (b.voltage or 0.0) > 0.0 or (b.level or 0.0) > 0.0]
    if not present:
        return None, None, None, None
    levels = [b.level for b in present if b.level is not None]
    volts = [b.voltage for b in present if b.voltage is not None]
    temps = [b.temperature for b in present if b.temperature is not None]
    charging = [b.charging for b in present if b.charging is not None]
    percentage = max(0.0, min(1.0, sum(levels) / len(levels) / 100.0)) if levels else None
    voltage = sum(volts) / len(volts) if volts else None
    temperature = max(temps) if temps else None
    return percentage, voltage, temperature, (any(charging) if charging else None)


def gps_status(fix_quality: float) -> int:
    """
    Return the sensor_msgs/NavSatStatus status of an NMEA fix quality.

    0 no fix (-1), 1 GPS (0), 2 differential (1, SBAS), 4/5 RTK (2, GBAS).
    """
    quality = int(round(finite_or(fix_quality, 0.0)))
    if quality <= 0:
        return -1
    if quality == 2:
        return 1
    if quality in (4, 5):
        return 2
    return 0
