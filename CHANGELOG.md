# Changelog

All notable changes to m20_ros2. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

### Fixed (final review)
- `m20_driver`: the zero burst after a hold (`navigation_mode`, `assist_mode`, `charge_start`,
  `sleep`, `sit_down`, ...), an overspeed stop or E006 starts at the next velocity period; before,
  it started at the event and a late period (or a `stop_burst_sec` shorter than a period) dropped
  it without a single zero. At least one zero now follows every motion.
- `m20_driver`: shutdown sends its three zeros whenever the last axis command was motion; before,
  only within a second of it, while `executor.shutdown()` first waits for blocked services.
- `m20_driver`: the regular mode tries of an episode stop 2 s after the first; before, a later
  motion in the same episode could send the rest a minute later.
- `m20_driver`: `stand_up`, `rl_control` and `regular_mode` no longer lift a hold that came in
  while they ran (an `emergency_stop` during `stand_up`): a hold counter decides.

### Safety (after an independent review)
- `m20_driver`: `cmd_vel` goes through a velocity gate. Axis commands go out only while armed and
  moving (non-zero after the 0.01 rounding), then a zero burst, then silence even while zero
  Twists keep coming (like the official client). Holds: `emergency_stop`, `sit_down`, `idle`,
  `damping`, `sleep`, `charge_start`, `navigation_mode`, `assist_mode` stop the stream before
  they send anything; `stand_up` / `rl_control` lift them once RL control is reported
  (`regular_mode` the mode holds). Disarm by the robot: not in RL control, no reports, hard
  emergency stop, charging, no Direction yet, overspeed. Takeover latch: the robot leaving
  regular mode after it was seen in regular mode (app, remote, robot, the mode services) stops
  axis commands and mode requests until it reports regular mode again. E006 (another client on
  the axis channel) yields until `cmd_vel` pauses. After any of these a stale stream never
  drives: `cmd_vel` has to be zero or absent once. Previously the next `cmd_vel` undid an
  emergency stop or a sit down, and the driver switched the robot back to regular mode after an
  operator took over.
- `m20_driver`: services succeed only when the robot confirmed them: no reply is `unconfirmed`
  (was success); `sit_down`, `idle`, `damping`, `release_emergency_stop` also wait for the
  reported state. `emergency_stop` sends the soft emergency stop three times and reports `soft
  e-stop NOT confirmed` (ERROR) without a reply or the state; it has its own callback group and
  aborts waiting services. A reply with another message ID reaches the oldest request of its
  kind; a hex-string ErrorCode is read.
- `m20_driver`: cmd_vel and the velocity timer, `emergency_stop` and the other services run in
  separate callback groups on five threads: blocking services no longer starve the watchdog, and
  a late timer still sends the zero burst.
- `m20_driver`: a NaN or infinity in any Twist field is a stop (was: only that field zeroed).
- `m20_driver`: the last reported Direction is kept (a stale basic report flipped X and Y);
  with `follow_direction` nothing but zeros is sent before a Direction is reported.
- `m20_driver`: gaits switch only in RL control, regular mode and with both batteries; `idle`
  and `damping` only from a sitting state; `stand_up` fails at once on a hard emergency stop or
  while charging.
- `m20_driver`: parameters are checked at start; an invalid value stops the driver before it
  contacts the robot (ERROR state, exit status 2, no respawn loop).

### Changed
- `m20_driver`: Roll/Pitch/Yaw unit from `angle_unit: auto | rad | deg` (default `auto`: found
  from Yaw against OmegaZ during a turn; no `imu` orientation until then, odometry integrates
  OmegaZ and continues without a jump). The 2*pi latch (wrong both ways) is gone.
- `m20_driver`: full scales per gait (`full_scale.<gait>.vx|vy|wz`), chosen by the reported gait;
  an unknown gait takes the largest. Overspeed guard: reported speed above 1.3 x `max_*` + 0.1
  for 0.5 s while driving stops the stream (ERROR). Speed estimates per gait.
- `m20_driver`: `cmd_rate_hz` 20 (the document's rate, was 12.5).
- `m20_driver`: `auto_regular_mode` asks for regular mode (3 tries) only when a motion episode
  starts and the robot has been in navigation mode since before regular mode was seen; never
  from assist mode or after a takeover (was: every 2 s while another mode was reported).
- `m20_driver`: GPS FixQuality 3 is a fix, 6, 7 and 8 no fix (NMEA).
- `m20_driver`: the heartbeat pauses while the velocity loop is hung for more than 1 s.
- `m20_driver`: the sleep settings are read before every `sleep` / `wake`.
- `m20_driver`: battery error bits named `rear (right)` and `front (left)`.
- `m20_driver`: shutdown sends three zeros after motion.
- `m20_description`: `display.launch.py` shows the model in any namespace.

### Added
- `m20_driver`: `release_emergency_stop` (MotionParam -2) and `charge_cancel` (Charge 3), both
  from the SDK, not in V1.3.0. Motion states -1, -2, 5 and the charge state 0x1000 have names
  (from the SDK). `direction_flips_reports` (default false) for robots that report
  LinearX/LinearY in their forward direction, with a diagnostics warning when the reported speed
  keeps the opposite sign. `transition_timeout_sec`. The `state` keys `velocity`, `heartbeat`,
  `axis_refusal`, `basic_age_sec`. Older named DeviceTemperature fields are read.
- `m20_driver`: the simulator can answer with a wrong message ID, stay silent, refuse axis
  commands with E006, change its usage mode on its own, and turn (Yaw follows OmegaZ, in rad or
  degrees).

### Removed
- `m20_driver`: parameters `full_scale_vx`, `full_scale_vy`, `full_scale_wz` (now per gait),
  `mode_refresh_sec`, `angles_in_degrees` (now `angle_unit`). SO_REUSEADDR on the socket (two
  drivers on one fixed port split the reports).

### Added
- `m20_driver`: the Lynx M20 inspection protocol V1.3.0-0 over UDP with JSON (16-byte header,
  message IDs, call status responses). Heartbeat from its own thread; `cmd_vel` as axis commands
  (ratios of `full_scale_*`, REP-103 signs, 12.5 Hz) in regular usage mode, switched to on its
  own, with a 0.5 s watchdog; X and Y negated while the forward direction is Back. Reports as
  `odom` (dead reckoning), `imu`, `joint_states` (12 leg angles, 4 wheels), `battery`,
  `gps/fix`, `state` and `diagnostics` (the document's error table, temperatures, power, speed
  estimates for calibration). Trigger services for the motion states (`stand_up` goes through
  Stand to RL control), gaits, usage modes, lights, autonomous charging, sleep and time sync.
  Roll/Pitch/Yaw read as rad, switched to degrees when an angle cannot be rad.
  `test/fake_robot.py` simulates the robot for the tests and for trying the driver by hand.
- `m20_description`: Deep Robotics' official model (deep_robotics_model `M20.urdf`, BSD-3) with
  its meshes, as xacro: official link and joint names, camera and LiDAR frames, `imu_link`.
  Description and display launch files, RViz config.
- `m20_bringup`: one launch for the driver and the URDF in the namespace `m20`, RViz on request.
- CI in a `ros:foxy` container: build, unit and node tests, an end-to-end smoke test of the
  launch file against the simulator, `check_urdf`, shellcheck.
