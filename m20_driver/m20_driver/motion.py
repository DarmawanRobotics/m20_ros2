"""Velocity streaming policy: which axis commands go to the robot, and when."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from m20_driver import protocol as p

EPSILON = 1e-3


@dataclass
class VelocityConfig:
    """Limits, scale and timing of the velocity stream."""

    max_vx: float = 1.0           # cmd_vel is clamped to these (m/s, m/s, rad/s)
    max_vy: float = 0.5
    max_wz: float = 1.0
    full_scale_vx: float = 5.0    # speed the robot runs at axis value 1.0 (calibrate)
    full_scale_vy: float = 2.0
    full_scale_wz: float = 3.0
    cmd_timeout_sec: float = 0.5
    stop_burst_sec: float = 0.5
    auto_regular_mode: bool = True
    mode_refresh_sec: float = 2.0
    invert_vy: bool = False
    invert_wz: bool = False


def clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]; a non-finite value becomes 0."""
    if not math.isfinite(value):
        return 0.0
    return max(-abs(limit), min(abs(limit), value))


def is_motion(cmd: Tuple[float, float, float]) -> bool:
    """Return True when any component asks for motion."""
    return any(abs(v) > EPSILON for v in cmd)


def ratio(value: float, full_scale: float) -> float:
    """Return the axis value for a speed: value / full scale, rounded and clamped."""
    if full_scale <= 0.0 or not math.isfinite(full_scale):
        return 0.0
    return p.rounded(value / full_scale)


class VelocityPolicy:
    """Streams cmd_vel while it is fresh, then zeros for a moment, then nothing."""

    def __init__(self, config: VelocityConfig):
        """Start idle: nothing is sent before the first command."""
        self.config = config
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp: Optional[float] = None
        self._last_motion: Optional[float] = None
        self._mode_sent: Optional[float] = None

    def command(self, vx: float, vy: float, wz: float, now: float):
        """Take a new cmd_vel (base_link, REP-103)."""
        c = self.config
        self._cmd = (clamp(vx, c.max_vx), clamp(vy, c.max_vy), clamp(wz, c.max_wz))
        self._stamp = now
        if is_motion(self._cmd):
            self._last_motion = now

    def stop(self, now: float):
        """Drop the current command; zeros are streamed until the stream runs out."""
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp = now

    @property
    def current(self) -> Tuple[float, float, float]:
        """Return the last command after clamping."""
        return self._cmd

    def fresh(self, now: float) -> bool:
        """Return True while the last command is younger than the timeout."""
        return self._stamp is not None and now - self._stamp <= self.config.cmd_timeout_sec

    def moving(self, now: float) -> bool:
        """Return True while a fresh command asks for motion."""
        return self.fresh(now) and is_motion(self._cmd)

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a non-zero command arrived within the window."""
        return self._last_motion is not None and now - self._last_motion <= window

    def sent_command(self, now: float) -> Tuple[float, float, float]:
        """Return what the stream sends now: the command, or zeros."""
        return self._cmd if self.fresh(now) else (0.0, 0.0, 0.0)

    def axis(self, vx: float, vy: float, wz: float, reverse: bool = False) -> dict:
        """
        Items of the axis command for a velocity (REP-103 in, REP-103 out).

        reverse: the robot's forward direction is Back (1.3.1.1 [2]: positive X then moves
        towards the hard emergency stop side), so X and Y are negated to stay in base_link. The
        yaw rate is the same physical turn either way.
        """
        c = self.config
        if reverse:
            vx, vy = -vx, -vy
        return p.axis_items(
            x=ratio(vx, c.full_scale_vx),
            y=ratio(-vy if c.invert_vy else vy, c.full_scale_vy),
            yaw=ratio(-wz if c.invert_wz else wz, c.full_scale_wz),
        )

    def tick(self, now: float, usage_mode: Optional[int],
             reverse: bool = False) -> List[Tuple[Tuple[int, int], dict]]:
        """
        Return the (message, items) requests for this period of the command timer.

        Axis commands only work in regular usage mode: with auto_regular_mode the regular mode
        is requested when motion starts in another (or an unknown) mode, and again every
        mode_refresh_sec while the robot reports another mode. reverse: see axis().
        """
        c = self.config
        if self._stamp is None:
            return []
        age = now - self._stamp
        if age <= c.cmd_timeout_sec:
            out = []
            if c.auto_regular_mode and is_motion(self._cmd) and usage_mode != p.USAGE_REGULAR:
                if self._mode_sent is None or now - self._mode_sent >= c.mode_refresh_sec:
                    out.append((p.USAGE_MODE, p.usage_mode_items(p.USAGE_REGULAR)))
                    self._mode_sent = now
            return out + [(p.AXIS, self.axis(*self._cmd, reverse=reverse))]
        if age <= c.cmd_timeout_sec + c.stop_burst_sec:
            return [(p.AXIS, self.axis(0.0, 0.0, 0.0))]
        self._mode_sent = None  # idle: the next motion checks the usage mode again
        return []


class SpeedEstimator:
    """Estimates the full-scale speeds from what was sent and what the robot reports."""

    def __init__(self, min_ratio: float = 0.2, smoothing: float = 0.1):
        """Only steady commands of at least min_ratio count; exponential smoothing."""
        self.min_ratio = min_ratio
        self.smoothing = smoothing
        self.estimates = {'vx': None, 'vy': None, 'wz': None}

    def update(self, sent: dict, measured: Tuple[Optional[float], Optional[float],
                                                 Optional[float]]):
        """Feed the last axis items and the reported (LinearX, LinearY, OmegaZ)."""
        for key, axis_key, value in zip(('vx', 'vy', 'wz'), ('X', 'Y', 'Yaw'), measured):
            r = sent.get(axis_key, 0.0)
            if value is None or abs(r) < self.min_ratio or not math.isfinite(value):
                continue
            sample = value / r
            if sample <= 0.0:
                continue  # moving the other way: a sign question, not a scale
            old = self.estimates[key]
            self.estimates[key] = sample if old is None else (
                old + self.smoothing * (sample - old))
