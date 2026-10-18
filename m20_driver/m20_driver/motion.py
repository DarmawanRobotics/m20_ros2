"""
Velocity gate: which axis commands go to the robot, and when (no ROS).

Axis commands (2/21) go out only while the gate is armed and the command is non-zero, then
zeros for stop_burst_sec, then nothing, even while zero Twists keep arriving (the official
client also sends a single zero). Motion means a non-zero axis value after the rounding to 0.01
that is actually sent.

- Holds, set by services before they send anything: no axis and no mode request (only the one
  zero burst) until stand_up or rl_control confirm RL control (regular_mode lifts the
  navigation_mode and assist_mode holds).
- Disarm by the robot: not in RL control, hard emergency stop, charging, Direction never
  reported (with follow_direction), overspeed.
- Takeover latch: the robot left regular usage mode after it was seen in regular mode (the app,
  the remote, the robot, or the navigation_mode / assist_mode services). Held until the robot
  reports regular mode again.
- E006 (another client holds the axis channel): yield until cmd_vel has been silent for
  cmd_timeout_sec + stop_burst_sec.

After any of these, motion is sent again only once cmd_vel has been zero or absent at least
once: a stale stream never drives off.
"""

from dataclasses import dataclass, field
import math
import threading
from typing import Dict, List, Optional, Tuple

from m20_driver import protocol as p

GAIT_NAMES = {
    p.GAIT_BASIC: 'basic',
    p.GAIT_STAIR: 'stair',
    p.GAIT_FLAT_AGILE: 'flat_agile',
    p.GAIT_STAIR_AGILE: 'stair_agile',
}
DEFAULT_FULL_SCALE = (5.0, 2.0, 3.0)    # m/s, m/s, rad/s at axis value 1: calibrate
MODE_ATTEMPTS = 3                        # regular mode requests at the start of an episode
MODE_RETRY_SEC = 0.5
OVERSPEED_FACTOR = 1.3                   # reported speed above 1.3 x max_* + 0.1 ...
OVERSPEED_MARGIN = 0.1
OVERSPEED_SEC = 0.5                      # ... for this long while driving: disarm
WARN_EVERY_SEC = 5.0
NAV_HOLDS = ('navigation_mode', 'assist_mode')
E006 = 0xE006


def default_full_scale() -> Dict[str, Tuple[float, float, float]]:
    """Return the same full scale for every gait."""
    return dict.fromkeys(GAIT_NAMES.values(), DEFAULT_FULL_SCALE)


@dataclass
class VelocityConfig:
    """Limits, scales and timing of the velocity stream."""

    max_vx: float = 1.0           # cmd_vel is clamped to these (m/s, m/s, rad/s)
    max_vy: float = 0.5
    max_wz: float = 1.0
    full_scale: Dict[str, Tuple[float, float, float]] = field(default_factory=default_full_scale)
    cmd_timeout_sec: float = 0.5
    stop_burst_sec: float = 0.5
    auto_regular_mode: bool = True
    invert_vy: bool = False
    invert_wz: bool = False
    follow_direction: bool = True

    @property
    def silence_sec(self) -> float:
        """Return how long cmd_vel must be silent to end an episode."""
        return self.cmd_timeout_sec + self.stop_burst_sec


@dataclass
class RobotView:
    """What the robot last reported; None: never reported."""

    usage_mode: Optional[int] = None
    motion_state: Optional[int] = None
    hes: Optional[int] = None
    charge: Optional[int] = None
    direction: Optional[int] = None
    gait: Optional[int] = None


def clamp(value: float, limit: float) -> float:
    """Clamp to [-limit, limit]."""
    return max(-abs(limit), min(abs(limit), value))


def ratio(value: float, full_scale: float) -> float:
    """Return the axis value for a speed: value / full scale, rounded and clamped."""
    if full_scale <= 0.0 or not math.isfinite(full_scale):
        return 0.0
    return p.rounded(value / full_scale)


def is_motion(items: dict) -> bool:
    """Return True when an axis command moves the robot (X, Y or Yaw not 0 after rounding)."""
    return any(items.get(key, 0.0) != 0.0 for key in ('X', 'Y', 'Yaw'))


@dataclass
class _Episode:
    """A run of non-zero commands; it ends after cmd_vel was silent for silence_sec."""

    request_modes: bool              # navigation mode at the start: ask for regular mode
    attempts: int = 0
    last_attempt: Optional[float] = None
    warned_mode: bool = False


class VelocityGate:
    """Decides the axis commands; every public method is thread safe."""

    def __init__(self, config: VelocityConfig):
        """Start disarmed: the first motion needs a robot in RL control and regular mode."""
        self.config = config
        self._lock = threading.Lock()
        self._cmd = (0.0, 0.0, 0.0)
        self._cmd_time: Optional[float] = None         # last Twist, zero or not
        self._motion_time: Optional[float] = None      # last Twist that was motion
        self._holds = set()
        self._armed = False
        self._disarm: Optional[str] = None             # shown while a stream is refused
        self._yield_e006 = False
        self._episode: Optional[_Episode] = None
        self._view = RobotView()
        self._regular_seen = False
        self._latched = False
        self._sent_motion = False
        self._driving = False
        self._burst_until: Optional[float] = None
        self._last_sent: Optional[dict] = None
        self._last_sent_time: Optional[float] = None
        self._over_since: Optional[float] = None
        self._overspeed = False
        self._events: List[Tuple[str, str]] = []
        self._warned_at: Dict[str, float] = {}

    # ------------------------------------------------------------ inputs

    def command(self, vx: float, vy: float, wz: float, now: float, finite: bool = True):
        """Take a cmd_vel (base_link, REP-103); finite=False: some field was NaN or infinite."""
        with self._lock:
            silent = self._silent(now)
            if finite and all(math.isfinite(v) for v in (vx, vy, wz)):
                c = self.config
                cmd = (clamp(vx, c.max_vx), clamp(vy, c.max_vy), clamp(wz, c.max_wz))
            else:
                cmd = (0.0, 0.0, 0.0)
                self._warn_every('nonfinite', now,
                                 'cmd_vel with NaN or infinity: treated as a stop')
            if silent:
                self._end_episode()
            self._cmd = cmd
            self._cmd_time = now
            if is_motion(self._axis(cmd)):
                self._motion_time = now
                if self._episode is None:
                    self._start_episode()
            elif self._blocked() is None:
                self._arm()

    def hold(self, reason: str, now: float):
        """Stop driving for a service (emergency_stop, sit_down, sleep, ...): zero burst only."""
        with self._lock:
            self._holds.add(reason)
            self._armed = False
            self._stop_motion(now)

    def release(self, reasons=None) -> bool:
        """Lift the given holds (all when None); motion then needs cmd_vel zero or absent first."""
        with self._lock:
            before = set(self._holds)
            if reasons is None:
                self._holds.clear()
            else:
                self._holds -= set(reasons)
            if self._holds != before:
                self._armed = False
                return True
            return False

    def axis_refused(self, code: int, now: float):
        """Take the robot's call status refusal of an axis command."""
        if code != E006:
            return
        with self._lock:
            if not self._yield_e006:
                self._yield_e006 = True
                self._armed = False
                self._stop_motion(now)
                self._events.append((
                    'warn', 'E006: another client holds the axis channel (2 s rule): cmd_vel is '
                    f'ignored until it has been silent for {self.config.silence_sec:.1f} s'))

    def observe_speed(self, now: float, vx: Optional[float], vy: Optional[float],
                      wz: Optional[float]):
        """Watch the reported speeds while driving: far above max_* means a wrong full scale."""
        with self._lock:
            c = self.config
            checks = (('LinearX', vx, c.max_vx), ('LinearY', vy, c.max_vy),
                      ('OmegaZ', wz, c.max_wz))
            over = [name for name, value, limit in checks
                    if value is not None and math.isfinite(value)
                    and abs(value) > OVERSPEED_FACTOR * limit + OVERSPEED_MARGIN]
            if not over:
                self._over_since = None
                self._overspeed = False        # gone; re-arming still needs zero or absent
                return
            if not self._driving:
                return
            if self._over_since is None:
                self._over_since = now
            elif not self._overspeed and now - self._over_since > OVERSPEED_SEC:
                self._overspeed = True
                self._armed = False
                self._stop_motion(now)
                self._events.append((
                    'error', f'overspeed: reported {", ".join(over)} above 1.3 x max_* + 0.1 for '
                    f'{OVERSPEED_SEC:.1f} s: driving stopped. Calibrate full_scale.'
                    f'{self._gait_name() or "<gait>"} (full_scale_*_estimate in diagnostics)'))

    # ------------------------------------------------------------ the stream

    def tick(self, now: float, view: RobotView) -> List[Tuple[Tuple[int, int], dict]]:
        """Return the (message, items) requests for this period of the command timer."""
        with self._lock:
            self._update_view(view, now)
            out = []
            if self._silent(now):
                self._end_episode()
            fresh = (self._cmd_time is not None
                     and now - self._cmd_time <= self.config.cmd_timeout_sec)
            items = self._axis(self._cmd if fresh else (0.0, 0.0, 0.0))
            moving = is_motion(items)
            blocked = self._blocked()
            drive = False
            if blocked is not None:
                self._armed = False
                if not moving:
                    self._disarm = None
                elif not (self._holds or self._latched or self._yield_e006):
                    self._disarm = blocked              # a stream the robot refuses to take
            elif not moving:
                self._arm()
            elif self._armed:
                drive = self._mode_allows_driving(now, out)
            elif self._disarm is None:
                self._disarm = 'stale_cmd_vel'          # waiting for a zero or a pause
            if drive:
                out.append((p.AXIS, items))
                self._sent_motion = True
                self._burst_until = None
                self._last_sent, self._last_sent_time = items, now
            else:
                self._stop_motion(now)
                if self._burst_until is not None:
                    if now < self._burst_until:
                        zero = p.axis_items()
                        out.append((p.AXIS, zero))
                        self._last_sent, self._last_sent_time = zero, now
                    else:
                        self._burst_until = None
            self._driving = drive
            return out

    # ------------------------------------------------------------ queries

    def status(self) -> Tuple[str, bool]:
        """Return (velocity state for the state topic, True when it deserves a WARN)."""
        with self._lock:
            other_holds = sorted(self._holds - set(NAV_HOLDS))
            if other_holds:
                return 'hold:' + '+'.join(other_holds), True
            if self._latched:
                mode = p.name_of(p.USAGE_MODES, self._view.usage_mode)
                return (f'takeover ({mode} mode): hand back in the app/remote or call '
                        'regular_mode'), True
            if self._holds:
                return 'hold:' + '+'.join(sorted(self._holds)), True
            if self._yield_e006:
                return 'axis_owner_other', True
            if self._disarm is not None:
                return f'disarmed:{self._disarm}', True
            return ('driving', False) if self._driving else ('idle', False)

    @property
    def last_axis(self) -> dict:
        """Return the last axis command sent (zeros before the first)."""
        with self._lock:
            return dict(self._last_sent) if self._last_sent else p.axis_items()

    def sent_within(self, now: float, window: float) -> bool:
        """Return True when an axis command went out within the window."""
        with self._lock:
            return self._last_sent_time is not None and now - self._last_sent_time <= window

    def recently_moved(self, now: float, window: float) -> bool:
        """Return True when a cmd_vel that was motion arrived within the window."""
        with self._lock:
            return self._motion_time is not None and now - self._motion_time <= window

    @property
    def driving(self) -> bool:
        """Return True while non-zero axis commands go out."""
        with self._lock:
            return self._driving

    @property
    def holds(self) -> Tuple[str, ...]:
        """Return the active holds."""
        with self._lock:
            return tuple(sorted(self._holds))

    def gait_name(self) -> Optional[str]:
        """Return the configured name of the reported gait, None when it has none."""
        with self._lock:
            return self._gait_name()

    def full_scale(self) -> Tuple[float, float, float]:
        """Return the full scale in use (the reported gait's, else the largest of each axis)."""
        with self._lock:
            return self._scale()

    def drain_events(self) -> List[Tuple[str, str]]:
        """Return and forget the (level, text) messages to log."""
        with self._lock:
            events, self._events = self._events, []
            return events

    # ------------------------------------------------------------ internals (lock held)

    def _silent(self, now: float) -> bool:
        return self._cmd_time is None or now - self._cmd_time >= self.config.silence_sec

    def _end_episode(self):
        self._episode = None
        self._yield_e006 = False

    def _start_episode(self):
        v, c = self._view, self.config
        request = (c.auto_regular_mode and v.usage_mode == p.USAGE_NAVIGATION
                   and not self._regular_seen and not self._latched and not self._holds
                   and v.charge not in p.CHARGING_STATES)
        self._episode = _Episode(request_modes=request)

    def _arm(self):
        self._armed = True
        self._disarm = None

    def _stop_motion(self, now: float):
        """Start the zero burst when the last command sent was motion."""
        if self._sent_motion:
            self._sent_motion = False
            self._burst_until = now + self.config.stop_burst_sec

    def _update_view(self, view: RobotView, now: float):
        self._view = view
        mode = view.usage_mode
        if mode == p.USAGE_REGULAR:
            if self._latched:
                self._latched = False
                self._armed = False
                self._events.append((
                    'info', 'regular mode reported again: takeover cleared; cmd_vel drives after '
                    'it was zero or absent once'))
            self._regular_seen = True
        elif mode is not None and self._regular_seen and not self._latched:
            self._latched = True
            self._armed = False
            self._stop_motion(now)
            self._events.append((
                'warn', f'the robot left regular mode ({p.name_of(p.USAGE_MODES, mode)} mode): '
                'takeover latched, no axis commands or mode requests until it reports regular '
                'mode again (hand back in the app/remote or call regular_mode)'))

    def _blocked(self) -> Optional[str]:
        """Return why nothing may be driven now, None when the robot may be driven."""
        if self._holds:
            return 'hold'
        if self._latched:
            return 'takeover'
        if self._yield_e006:
            return 'axis_owner_other'
        v = self._view
        if v.hes == 1:
            return 'hard_estop'
        if v.charge in p.CHARGING_STATES:
            return 'charging'
        if v.motion_state != p.MOTION_RL_CONTROL:
            return 'not_rl_control' if v.motion_state is not None else 'no_motion_state'
        if self.config.follow_direction and v.direction is None:
            return 'direction_unknown'
        if self._overspeed:
            return 'overspeed'
        return None

    def _mode_allows_driving(self, now: float, out: list) -> bool:
        """Return True in regular mode; at the start of an episode ask for it once (3 tries)."""
        mode = self._view.usage_mode
        if mode == p.USAGE_REGULAR:
            return True
        episode = self._episode
        if episode is None:
            return False
        if episode.request_modes and mode == p.USAGE_NAVIGATION:
            if episode.attempts < MODE_ATTEMPTS:
                if episode.last_attempt is None or now - episode.last_attempt >= MODE_RETRY_SEC:
                    out.append((p.USAGE_MODE, p.usage_mode_items(p.USAGE_REGULAR)))
                    episode.attempts += 1
                    episode.last_attempt = now
                return False
            if now - episode.last_attempt < MODE_RETRY_SEC:
                return False
        if mode is not None and not episode.warned_mode:
            episode.warned_mode = True
            name = p.name_of(p.USAGE_MODES, mode)
            reason = ('it did not switch after 3 requests' if episode.request_modes
                      else 'axis commands need regular mode')
            self._events.append(('warn', f'usage mode {name}: {reason}; call regular_mode'))
        return False

    def _gait_name(self) -> Optional[str]:
        name = GAIT_NAMES.get(self._view.gait)
        return name if name in self.config.full_scale else None

    def _scale(self) -> Tuple[float, float, float]:
        name = self._gait_name()
        if name is not None:
            return self.config.full_scale[name]
        scales = list(self.config.full_scale.values()) or [DEFAULT_FULL_SCALE]
        return tuple(max(scale[i] for scale in scales) for i in range(3))

    def _axis(self, cmd: Tuple[float, float, float]) -> dict:
        """Return the axis Items for a velocity (REP-103 in, REP-103 out)."""
        c = self.config
        vx, vy, wz = cmd
        if c.follow_direction and self._view.direction == 1:
            vx, vy = -vx, -vy           # 1.3.1.1 [2]: forward is then the hard e-stop side
        fx, fy, fw = self._scale()
        return p.axis_items(
            x=ratio(vx, fx),
            y=ratio(-vy if c.invert_vy else vy, fy),
            yaw=ratio(-wz if c.invert_wz else wz, fw),
        )

    def _warn_every(self, key: str, now: float, text: str):
        last = self._warned_at.get(key)
        if last is None or now - last >= WARN_EVERY_SEC:
            self._warned_at[key] = now
            self._events.append(('warn', text))


class SpeedEstimator:
    """Estimates the full scales per gait from what was sent and what the robot reports."""

    def __init__(self, min_ratio: float = 0.2, smoothing: float = 0.1):
        """Only steady commands of at least min_ratio count; exponential smoothing."""
        self.min_ratio = min_ratio
        self.smoothing = smoothing
        self.estimates: Dict[str, Dict[str, Optional[float]]] = {}

    def update(self, gait: Optional[str], sent: dict,
               measured: Tuple[Optional[float], Optional[float], Optional[float]]):
        """Feed the last axis items and the reported (LinearX, LinearY, OmegaZ) in a gait."""
        if gait is None:
            return
        estimates = self.estimates.setdefault(gait, {'vx': None, 'vy': None, 'wz': None})
        for key, axis_key, value in zip(('vx', 'vy', 'wz'), ('X', 'Y', 'Yaw'), measured):
            r = sent.get(axis_key, 0.0)
            if value is None or abs(r) < self.min_ratio or not math.isfinite(value):
                continue
            sample = value / r
            if sample <= 0.0:
                continue  # moving the other way: a sign question, not a scale
            old = estimates[key]
            estimates[key] = sample if old is None else old + self.smoothing * (sample - old)
