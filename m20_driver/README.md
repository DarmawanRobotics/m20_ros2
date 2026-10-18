# m20_driver

ROS 2 driver of the Deep Robotics Lynx M20. It speaks the robot's inspection protocol (Deep
Robotics *Inspection Communication Protocol* V1.3.0-0, UDP with JSON): `cmd_vel` through a
velocity gate to axis commands in regular usage mode, the status reports as ROS topics, and
every robot command as a `std_srvs/Trigger` service that succeeds only when the robot confirmed
it.

## Nodes

| Node | Executable | |
|---|---|---|
| `m20_driver` | `driver_node` | UDP link to the robot, ROS topics and services |

## Usage

```bash
ros2 launch m20_driver driver.launch.py namespace:=m20
ros2 topic echo /m20/state
ros2 service call /m20/stand_up std_srvs/srv/Trigger
```

Without a robot, against the simulator in `test/` (the same one the tests use):

```bash
python3 $(ros2 pkg prefix m20_driver)/share/m20_driver/test/fake_robot.py --port 30000
ros2 launch m20_bringup bringup.launch.py robot_ip:=127.0.0.1
```

## Topics (in the launch namespace, `m20` by default)

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | `base_link`, REP-103: `linear.x` forward, `linear.y` left, `angular.z` counter-clockwise; goes through the [velocity gate](#velocity) |
| pub | `odom` | `nav_msgs/Odometry` | dead reckoning: `LinearX`/`LinearY` of the motion report (10 Hz) turned by the heading (the reported `Yaw` once its unit is known, the integrated `OmegaZ` before); twist in `base_link`; **no TF** unless `publish_tf` |
| pub | `imu` | `sensor_msgs/Imu` | `Roll`/`Pitch`/`Yaw` as a quaternion once the angle unit is known (`orientation_covariance[0] = -1` before), `OmegaZ` (and `OmegaX`/`OmegaY`, `AccX`/`AccY`/`AccZ` when the firmware sends them), frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | `fl_hipx_joint` ... `hr_wheel_joint` (the official model's names): 12 leg angles (velocity NaN: not reported); the 4 wheels as speed, their angle integrated from it |
| pub | `battery` | `sensor_msgs/BatteryState` | both batteries: mean `percentage` (0–1) and voltage, hottest temperature, charging |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` | the GPS group of the device report; covariance from HDOP/VDOP; altitude reference (MSL or ellipsoid) not documented |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | readable state as key/values (below), 2 Hz |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | `m20_driver` (state, link counters, speed estimates per gait), `m20 errors`, `m20 temperatures`, `m20 power`, 1 Hz |

`state` keys: `connected`, `age_sec`, `motion_state`, `velocity` (below), `angle_unit`,
`errors`, `cmd_axis` (the last X Y Yaw sent), `heartbeat` (`running` / `paused`),
`axis_refusal` (the last refused axis command), `gait`, `usage_mode`, `charge`, `hard_estop`,
`direction`, `obstacle_avoidance`, `power_management`, `sleep`, `version`, `basic_age_sec`,
`height_m`, `remain_km`, `battery`, `lights`. Level ERROR without reports, on a hard or soft
emergency stop or with invalid parameters; WARN while `velocity` is not `driving` or `idle`,
with robot errors, a low battery, while sleeping or with paused heartbeats.

`velocity`: `driving`, `idle`, `hold:<service>`, `disarmed:<reason>`
(`not_rl_control`, `no_motion_state`, `hard_estop`, `charging`, `direction_unknown`,
`overspeed`, `stale_cmd_vel`), `takeover (<mode> mode): hand back in the app/remote or call
regular_mode`, `axis_owner_other`.

## Services (`std_srvs/Trigger`)

Each service succeeds only when the robot confirmed it: its call status response (1.5) with
ErrorCode 0, and for motion states the state it then reports. No answer within
`response_timeout_sec` fails with `unconfirmed: no reply within 1.0 s`; a refusal fails with
the code and its meaning (e.g. `robot refused: 0xE008 not allowed operation`). A refusal
answered with another message ID still reaches the request of its kind.

| Service | Sends | Succeeds when | Notes |
|---|---|---|---|
| `stand_up` | 2/22 Stand (1), then RL Control (17) | RL control reported | refused at once on a hard emergency stop or while charging; lifts every hold; then `stand_up_gait` if set |
| `rl_control` | 2/22 RL Control (17) | RL control reported | from standing; lifts every hold |
| `sit_down` | 2/22 Sit (4) | Sit reported | hold; refused while walking (`cmd_vel` within 1 s or the reported speed above the official client's thresholds) |
| `idle`, `damping` | 2/22 Idle (0), Power-on Damping (3) | the state reported | hold; only from sit, idle, soft emergency stop or damping ("sit down first") |
| `emergency_stop` | zero axis, then 2/22 Soft Emergency Stop (2) three times 50 ms apart | a reply OK or `soft_emergency_stop` reported within 1.5 s | hold first; its own callback group, it aborts any service waiting; otherwise `soft e-stop NOT confirmed` and an ERROR log |
| `release_emergency_stop` | 2/22 MotionParam −2 (from the SDK, not in V1.3.0) | the robot leaves the soft emergency stop | the `cmd_vel` hold stays until `stand_up` or `rl_control` |
| `gait_basic`, `gait_stair` | 2/23 `0x1001`, `0x1003` | reply | Standard Motion Mode; only in RL control, regular mode and with both batteries (the SDK's rule) |
| `gait_flat_agile`, `gait_stair_agile` | 2/23 `0x3002`, `0x3003` | reply | Agile Motion Mode (the document: better speed response, suits autonomy); same conditions |
| `regular_mode` | 1101/5 Mode 0 | reply | lifts a `navigation_mode` / `assist_mode` hold; a takeover ends when the robot reports regular mode |
| `navigation_mode`, `assist_mode` | 1101/5 Mode 1, 2 | reply | hold, and a takeover once the robot reports the mode |
| `lights_on`, `lights_off` | 1101/2 Front and Back 1 / 0 | reply | |
| `charge_start` | 2/24 Charge 1 | reply | hold; the robot docks by itself |
| `charge_stop` | 2/24 Charge 0 | reply | the SDK: end or leave charging, valid while charging or on the dock (states 2 and 5) |
| `charge_cancel` | 2/24 Charge 3 (from the SDK, not in V1.3.0) | reply | cancel going to the dock (state 1) |
| `sleep`, `wake` | 1101/6 Sleep true / false | reply | Auto and Time as the robot reports them (1101/7, read before every call), else `sleep_auto`, `sleep_minutes`; `sleep` holds, `wake` does not lift it |
| `sync_time` | 1101/15 with this computer's local time | reply | |

Holds are set before anything is sent and stay when the command fails.

## Velocity

`cmd_vel` is clamped to `max_vx`, `max_vy`, `max_wz` and becomes the axis command 2/21: X, Y and
Yaw are the ratios `v / full_scale.<gait>.*` of the reported gait (an unknown gait uses the
largest value of each axis, the slowest real speed), rounded to 0.01 and clamped to [-1, 1],
with REP-103 signs (the document: X forward, Y left, Yaw by the right-hand rule; the official
client sends them so). Z, Roll and Pitch stay 0: only X, Y and Yaw act in RL control. A Twist
with NaN or infinity in any of its six fields is a stop (throttled WARN).

The gate sends axis commands at `cmd_rate_hz` (20 Hz, the document's rate) only while it is
armed and the command moves the robot (non-zero after the rounding: 0.002 m/s is no motion).
After the last motion it sends zeros for `stop_burst_sec`, then nothing, even while zero Twists
keep arriving: the official client sends a single zero too, and the robot accepts axis commands
from one client within 2 s only (E006). A stream that goes stale (`cmd_timeout_sec`) gets the
same zero burst, also when the timer resumes late.

- **Holds** (set by `emergency_stop`, `sit_down`, `idle`, `damping`, `sleep`, `charge_start`,
  `navigation_mode`, `assist_mode` before they send anything): no axis command and no mode
  request, only the one zero burst. `stand_up` and `rl_control` lift every hold once RL control
  is reported; `regular_mode` lifts only the `navigation_mode` / `assist_mode` hold;
  `release_emergency_stop` and `wake` lift nothing. A releasing service never lifts a hold that
  came in while it ran (an `emergency_stop` during `stand_up` stays).
- **Disarm** by the robot: not in RL control, no reports, hard emergency stop, charging
  (`Charge` 1, 2, 3), no `Direction` reported yet (with `follow_direction`), overspeed (a
  reported speed above 1.3 × `max_*` + 0.1 for 0.5 s while driving: a wrong full scale,
  ERROR).
- **Takeover latch**: the robot leaves regular usage mode after it was seen in regular mode —
  the app, the remote, the robot itself, or the `navigation_mode` / `assist_mode` services,
  whether `cmd_vel` moves or not. No axis command and no mode request until the robot reports
  regular mode again (hand back in the app or the remote, or call `regular_mode`); zeros and
  silence do not end it.
- **E006**: the call status of every axis command is read. E006 (another client holds the axis
  channel) disarms until `cmd_vel` has been silent for `cmd_timeout_sec + stop_burst_sec`; the
  next motion tries again. `state` says `axis_owner_other` (WARN).

The zero burst starts at the first period that does not drive, so at least one zero follows
any motion, however late that period runs. After any of these, motion is sent again only once
`cmd_vel` has been zero or absent at least once (`disarmed:stale_cmd_vel` until then): a stale
stream never drives off. The last reported
`Direction` is kept for good; while it is Back, X and Y go out negated so that `cmd_vel` stays
in `base_link` (1.3.1.1 [2]: positive X then moves towards the hard emergency stop side;
`follow_direction`).

With `auto_regular_mode` the driver asks for regular mode (up to 3 times, 0.5 s apart, within
2 s of the first try) only when a motion episode starts (the first motion after `cmd_vel` was silent for `cmd_timeout_sec +
stop_burst_sec`) while the robot has been in navigation mode since before regular mode was ever
seen (e.g. after boot) and nothing holds it: never from assist mode or an unknown mode, never
while charging, never after a takeover. Otherwise it warns once that `regular_mode` must be
called.

The protocol gives no speed for an axis value of 1, and a gait's speed range depends on the gait
(1.2.5: ratio of the gait's maximum). The `full_scale.<gait>.*` defaults (5 m/s, 2 m/s, 3 rad/s)
are deliberately high, so an uncalibrated robot runs **slower** than asked. Drive steadily in
each gait and read `full_scale.<gait>.vx_estimate` (`vy`, `wz`) in `diagnostics`: reported speed
divided by the sent ratio. Set those values in your copy of the parameters.

cmd_vel, the velocity timer, `emergency_stop` and the other services run in separate callback
groups on five executor threads: a service waiting for the robot never delays the watchdog or
the emergency stop. On shutdown the driver sends three zeros when the last axis command it sent
was motion, however long ago (a blocked service can stop the timer for seconds first).

## Reports

| Report | Rate | Becomes |
|---|---|---|
| heartbeat 100/100 | sent at `heartbeat_hz` from its own thread; paused while the velocity loop is hung for more than 1 s (WARN), so the robot notices a dead controller | the robot reports to the sender of the heartbeat: nothing to configure on the robot |
| basic status 1002/6 | 2 Hz | `state` (motion state, gait, usage mode, charge, hard e-stop, direction, sleep, ...); the values are kept when it stops |
| motion status 1002/4 | 10 Hz | `odom`, `imu`, `joint_states`, `state` |
| device status 1002/5 | 2 Hz | `battery`, `gps/fix`, `diagnostics` (temperatures, also the older named fields, CPU, lights) |
| abnormal status 1002/3 | 2 Hz | `diagnostics` (`m20 errors`, the document's error table with the joints or batteries named) |

**Angles:** the document gives `Roll`/`Pitch`/`Yaw` in rad, the official Android client
(`MotionStatusInfo`, `MotionFragment`) treats them as degrees. `angle_unit: auto` (default)
compares the unwrapped change of `Yaw` with the integrated `OmegaZ` during turns faster than
0.3 rad/s: about 57.3 times means degrees, about 1 radians; five agreeing windows decide, the
unit is kept for the run and logged (INFO, suggesting to set it). Until then `imu` has no
orientation and `odom` integrates `OmegaZ`; at the decision the odometry heading continues
without a jump. Set `angle_unit: rad` or `deg` once you know it.

**Batteries:** from `BatteryList` (entry 0 = rear, 1 = front, as the official client names
them), else from `BatteryStatus` (`left`, `right`). The error bits name them `rear (right)` and
`front (left)` (1.3.1.3 [4], 1.3.2 [7]: right is the hard emergency stop side).

**GPS:** `FixQuality` read as NMEA: 1 and 3 fix, 2 SBAS (differential), 4 and 5 GBAS (RTK), the
rest no fix.

**Direction reports:** `follow_direction` turns the commands; whether the robot also reports
`LinearX`/`LinearY` in its forward direction is not documented. `direction_flips_reports: true`
negates them while Direction=Back (odometry and the speed estimates); the diagnostics warn when,
with Direction=Back and driving, the reported speed keeps the opposite sign of `cmd_vel` for 2 s.

## Parameters

[config/driver_params.yaml](config/driver_params.yaml), every one commented. They are checked at
start: an invalid value (a rate, timeout, limit or full scale not above 0, `cmd_timeout_sec`
shorter than two command periods, a bad IP address or port, an unknown `angle_unit`) stops the
driver before it contacts the robot; `state` says why (ERROR), and the process exits with status
2 when stopped, so the launch file's respawn does not spin. `heartbeat_hz` below 1, `max_*` above
a full scale and `sleep_minutes` outside [5, 30] only warn.

| Parameter | Default | |
|---|---|---|
| `robot_ip`, `robot_port` | `10.21.31.103`, `30000` | the robot's UDP server (1.1.2) |
| `local_ip`, `local_port` | `0.0.0.0`, `0` | this end (any free port: the robot answers the heartbeat sender) |
| `heartbeat_hz` | 5.0 | the document: at least 1 Hz |
| `cmd_rate_hz`, `cmd_timeout_sec`, `stop_burst_sec` | 20.0, 0.5, 0.5 | velocity stream, watchdog, zero burst |
| `auto_regular_mode` | true | regular mode at the start of an episode, from navigation mode since boot only |
| `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.0 | `cmd_vel` clamp; the overspeed guard uses them |
| `full_scale.<gait>.vx`, `.vy`, `.wz` | 5.0, 2.0, 3.0 for `basic`, `stair`, `flat_agile`, `stair_agile` | speed at axis value 1 per gait: calibrate |
| `invert_vy`, `invert_wz` | false | sign fixes, [Verify on the real robot](../README.md#verify-on-the-real-robot) |
| `follow_direction` | true | negate X and Y while the forward direction is Back |
| `direction_flips_reports` | false | negate the reported LinearX/LinearY while Back |
| `angle_unit` | `auto` | `auto`, `rad`, `deg` |
| `odom_frame`, `base_frame`, `imu_frame`, `gps_frame` | `odom`, `base_link`, `imu_link`, `base_link` | message frames |
| `publish_tf` | false | `odom` → `base_link` from the dead reckoning |
| `state_hz`, `state_timeout_sec` | 2.0, 1.5 | no report for this long: disconnected (and not drivable) |
| `response_timeout_sec` | 1.0 | wait for the call status response |
| `stand_timeout_sec`, `rl_timeout_sec`, `stand_up_gait` | 10.0, 5.0, `''` | `stand_up` |
| `transition_timeout_sec` | 10.0 | `sit_down`, `idle`, `damping`, `release_emergency_stop` |
| `sleep_auto`, `sleep_minutes` | false, 10 | `sleep` / `wake` when the robot does not report its settings |
| `low_battery` | 0.2 | WARN below |

## Launch

`launch/driver.launch.py`:

| Argument | Default | |
|---|---|---|
| `namespace` | `m20` | topics and services |
| `params_file` | [config/driver_params.yaml](config/driver_params.yaml) | a copy with this robot's settings |

The driver with the URDF starts from [m20_bringup](../m20_bringup/README.md).

## Test

```bash
colcon test --packages-select m20_driver && colcon test-result --verbose
```

The codec is checked against the document's own header layout and JSON examples and against
the official Android client's framing (header, axis rounding). Every rule of the velocity gate
has a unit test. The link and the node run against `test/fake_robot.py` on the loopback
interface, which can also misbehave: answer with a wrong message ID, stay silent, refuse axis
commands with E006, change its usage mode on its own, report degrees. `test/smoke_test.sh` runs
the installed launch file against the simulator: the `ros2` command line checks the graph and a
service call, `test/smoke_check.py` (one node) the topics, the services, `cmd_vel` and its
watchdog, and the emergency stop holding `cmd_vel` (CI does both).
