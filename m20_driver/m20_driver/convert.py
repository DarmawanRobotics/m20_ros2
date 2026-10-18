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


class AngleUnitDetector:
    """
    Unit of the reported Roll/Pitch/Yaw: configured, or found from the yaw rate.

    The protocol document says rad, the official Android client calls them degrees and shows
    them with a degree sign. In auto mode the unwrapped change of Yaw is compared with the
    integrated OmegaZ (rad/s) during steady turns (|OmegaZ| > 0.3 rad/s): a ratio near 57.3 means
    degrees, near 1 radians. Five agreeing windows decide, then the unit is kept for the run.
    """

    MIN_RATE = 0.3          # rad/s: only turns this fast count
    WINDOW = 0.15           # rad of integrated OmegaZ per comparison
    TOLERANCE = 0.2         # relative
    VOTES = 5
    MAX_STEP_SEC = 0.5

    def __init__(self, mode: str = 'auto'):
        """Start with mode auto (unknown until found), rad or deg."""
        if mode not in ('auto', 'rad', 'deg'):
            raise ValueError(f'angle unit {mode!r}: auto, rad or deg')
        self.mode = mode
        self.unit: Optional[str] = None if mode == 'auto' else mode
        self._last: Optional[Tuple[float, float, float]] = None   # time, yaw, omega_z
        self._sums = [0.0, 0.0, 0.0]                              # deg, rad, integrated omega
        self._streak: Tuple[Optional[str], int] = (None, 0)

    @property
    def name(self) -> str:
        """Return the unit for the state: rad, deg, or 'rad|deg|unknown (auto)'."""
        if self.mode != 'auto':
            return self.mode
        return f'{self.unit or "unknown"} (auto)'

    def _restart(self):
        self._sums = [0.0, 0.0, 0.0]

    def observe(self, now: float, yaw: Optional[float], omega_z: Optional[float]) -> Optional[str]:
        """Feed one motion report; return the unit when this report decided it."""
        if self.unit is not None:
            return None
        if yaw is None or omega_z is None or not (math.isfinite(yaw) and math.isfinite(omega_z)):
            self._last = None
            self._restart()
            return None
        last, self._last = self._last, (now, yaw, omega_z)
        if last is None or abs(omega_z) <= self.MIN_RATE or abs(last[2]) <= self.MIN_RATE:
            self._restart()
            return None
        dt = now - last[0]
        if not 0.0 < dt <= self.MAX_STEP_SEC:
            self._restart()
            return None
        change = yaw - last[1]
        self._sums[0] += math.remainder(change, 360.0)
        self._sums[1] += math.remainder(change, TWO_PI)
        self._sums[2] += 0.5 * (omega_z + last[2]) * dt
        if abs(self._sums[2]) < self.WINDOW:
            return None
        degrees, radians, turned = self._sums
        self._restart()
        if abs(degrees / turned / math.degrees(1.0) - 1.0) < self.TOLERANCE:
            vote = 'deg'
        elif abs(radians / turned - 1.0) < self.TOLERANCE:
            vote = 'rad'
        else:
            self._streak = (None, 0)
            return None
        unit, count = self._streak
        count = count + 1 if unit == vote else 1
        self._streak = (vote, count)
        if count >= self.VOTES:
            self.unit = vote
            return vote
        return None

    def radians(self, value: Optional[float]) -> Optional[float]:
        """Return a reported angle in radians, None while the unit is unknown."""
        if self.unit is None:
            return None
        value = finite_or(value, 0.0)
        return math.radians(value) if self.unit == 'deg' else value


class Odometry2D:
    """
    Planar dead reckoning from body velocities and a heading.

    The heading is the reported Yaw once its unit is known, before that the integrated OmegaZ.
    When the reported heading takes over, it is offset to continue where the integration was,
    so the published heading never jumps.
    """

    def __init__(self, max_step_sec: float = 0.5):
        """Start at the origin; gaps longer than max_step_sec are not integrated."""
        self.max_step_sec = max_step_sec
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._last: Optional[float] = None
        self._offset: Optional[float] = None

    def reset(self):
        """Go back to the origin."""
        self.x = self.y = self.yaw = 0.0
        self._last = None
        self._offset = None

    def update(self, now: float, vx: float, vy: float, heading: Optional[float] = None,
               rate: Optional[float] = None) -> bool:
        """
        Integrate body velocities (m/s) over the time since the last update.

        heading (rad): the reported Yaw, None while its unit is unknown; rate (rad/s): OmegaZ,
        integrated while there is no heading. Return False when the step was skipped (first
        sample, a gap, or time going backwards).
        """
        last, self._last = self._last, now
        dt = None if last is None else now - last
        valid = dt is not None and 0.0 < dt <= self.max_step_sec
        if heading is not None:
            if self._offset is None:
                self._offset = self.yaw - heading if last is not None else 0.0
            self.yaw = math.remainder(heading + self._offset, TWO_PI)
        elif valid and rate is not None and math.isfinite(rate):
            self.yaw = math.remainder(self.yaw + rate * dt, TWO_PI)
        if not valid:
            return False
        c, s = math.cos(self.yaw), math.sin(self.yaw)
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

    1 GPS and 3 PPS: fix (0); 2 differential: SBAS (1); 4 RTK fixed and 5 RTK float: GBAS (2);
    anything else (0 invalid, 6 dead reckoning, 7 manual, 8 simulation): no fix (-1).
    """
    quality = int(round(finite_or(fix_quality, 0.0)))
    if quality in (1, 3):
        return 0
    if quality == 2:
        return 1
    if quality in (4, 5):
        return 2
    return -1
