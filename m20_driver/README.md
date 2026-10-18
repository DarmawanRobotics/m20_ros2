# m20_driver

ROS 2 driver of the Deep Robotics Lynx M20. It speaks the robot's inspection protocol (Deep
Robotics *Inspection Communication Protocol* V1.3.0-0, UDP with JSON): `cmd_vel` as axis
commands in regular usage mode, the status reports as ROS topics, and every robot command as a
`std_srvs/Trigger` service that waits for the robot's answer.

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
| sub | `cmd_vel` | `geometry_msgs/Twist` | `base_link`, REP-103: `linear.x` forward, `linear.y` left, `angular.z` counter-clockwise; zeros after `cmd_timeout_sec` without a command |
| pub | `odom` | `nav_msgs/Odometry` | dead reckoning: `LinearX`/`LinearY` of the motion report (10 Hz) turned by the reported `Yaw`; twist in `base_link`; **no TF** unless `publish_tf` |
| pub | `imu` | `sensor_msgs/Imu` | `Roll`/`Pitch`/`Yaw` as a quaternion, `OmegaZ` (and `OmegaX`/`OmegaY`, `AccX`/`AccY`/`AccZ` when the firmware sends them), frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | `fl_hipx_joint` ... `hr_wheel_joint` (the official model's names): 12 leg angles; the 4 wheels as speed, their angle integrated from it |
| pub | `battery` | `sensor_msgs/BatteryState` | both batteries: mean `percentage` (0–1) and voltage, hottest temperature, charging |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` | the GPS group of the device report; covariance from HDOP/VDOP |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | readable state as key/values (below), 2 Hz |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | `m20_driver` (state, link counters, speed estimates), `m20 errors`, `m20 temperatures`, `m20 power`, 1 Hz |

`state` keys: `connected`, `age_sec`, `motion_state`, `angle_unit`, `errors`, `cmd_axis` (the
last X Y Yaw sent), `gait`, `usage_mode`, `charge`, `hard_estop`, `direction`,
`obstacle_avoidance`, `power_management`, `sleep`, `version`, `height_m`, `remain_km`,
`battery`, `lights`. Level ERROR without reports, on a hard or soft emergency stop; WARN with
robot errors, a low battery or while sleeping.

## Services (`std_srvs/Trigger`)

| Service | Sends | Notes |
|---|---|---|
| `stand_up` | motion state 2/22 Stand (1), then RL Control (17) | waits for each state in the reports (`stand_timeout_sec`, `rl_timeout_sec`); nothing to do in RL control; then `stand_up_gait` if set |
| `sit_down`, `idle`, `damping` | 2/22 Sit (4), Idle (0), Power-on Damping (3) | refused while `cmd_vel` moved the robot within the last second |
| `rl_control` | 2/22 RL Control (17) | from standing |
| `emergency_stop` | zero axis command, then 2/22 Soft Emergency Stop (2) | also drops the current `cmd_vel` |
| `gait_basic`, `gait_stair` | 2/23 `0x1001`, `0x1003` | Standard Motion Mode |
| `gait_flat_agile`, `gait_stair_agile` | 2/23 `0x3002`, `0x3003` | Agile Motion Mode (the document: better speed response, suits autonomy) |
| `regular_mode`, `navigation_mode`, `assist_mode` | usage mode 1101/5 Mode 0, 1, 2 | axis commands (`cmd_vel`) need regular mode |
| `lights_on`, `lights_off` | 1101/2 Front and Back 1 / 0 | |
| `charge_start`, `charge_stop` | 2/24 Charge 1 / 0 | autonomous charging (the clear command 2 is not offered) |
| `sleep`, `wake` | 1101/6 Sleep true / false | Auto and Time as the robot reports them (1101/7), else `sleep_auto`, `sleep_minutes` |
| `sync_time` | 1101/15 with this computer's local time | |

Every service waits `response_timeout_sec` for the robot's call status response (1.5): an
ErrorCode other than 0 fails the call with the code and its meaning (e.g. `0xE008 not allowed
operation`). Without an answer the call succeeds with `(no reply within 1.0 s)` in the message.

## Velocity

`cmd_vel` is clamped to `max_vx`, `max_vy`, `max_wz` and sent as the axis command 2/21 at
`cmd_rate_hz` (12.5 Hz, the official client's 80 ms; the document recommends 20 Hz): X, Y and
Yaw are the ratios `v / full_scale_*`, rounded to 0.01 and clamped to [-1, 1], with REP-103 signs
(the document: X forward, Y left, Yaw by the right-hand rule; the official client sends them so).
Z, Roll and Pitch stay 0: the document says only X, Y and Yaw act in RL control. While the
robot reports its forward direction as Back (basic status `Direction` 1: positive X then moves
towards the hard emergency stop side), X and Y go out negated so `cmd_vel` stays in `base_link`
(`follow_direction`, logged when it happens).

The protocol gives no speed for an axis value of 1. The `full_scale_*` defaults (5 m/s, 2 m/s,
3 rad/s) are deliberately high, so an uncalibrated robot runs **slower** than asked. Drive
steadily and read `full_scale_vx_estimate` (`_vy_`, `_wz_`) in `diagnostics`: reported speed
divided by the sent ratio. Set those values in your copy of the parameters.

Axis commands only work in **regular** usage mode and move the robot in **RL control**
(`stand_up` gets it there). With `auto_regular_mode` the driver requests regular mode when
motion starts in another mode, and again every `mode_refresh_sec` while the robot reports
another. When `cmd_vel` goes quiet for `cmd_timeout_sec` the driver sends zeros for
`stop_burst_sec`, then nothing. The robot takes axis commands from one client only (`0xE006
mismatched request client`): close the official app's control first.

## Reports

| Report | Rate | Becomes |
|---|---|---|
| heartbeat 100/100 | sent at `heartbeat_hz` from its own thread | the robot reports to the sender of the heartbeat: nothing to configure on the robot |
| basic status 1002/6 | 2 Hz | `state` (motion state, gait, usage mode, charge, hard e-stop, sleep, ...) |
| motion status 1002/4 | 10 Hz | `odom`, `imu`, `joint_states`, `state` |
| device status 1002/5 | 2 Hz | `battery`, `gps/fix`, `diagnostics` (temperatures, CPU, lights) |
| abnormal status 1002/3 | 2 Hz | `diagnostics` (`m20 errors`, the document's error table with the joints or batteries named) |

**Angles:** the document gives `Roll`/`Pitch`/`Yaw` in rad, the official Android client calls
them degrees. The driver reads rad (`angles_in_degrees: false`) and switches to degrees by
itself the first time an angle above 2π arrives (`state.angle_unit`, logged once).

**Batteries:** from `BatteryList` (entry 0 = rear, 1 = front, as the official client names
them), else from `BatteryStatus` (`left`, `right`).

**GPS:** `FixQuality` read as NMEA: 0 no fix, 1 GPS, 2 differential, 4/5 RTK.

## Parameters

[config/driver_params.yaml](config/driver_params.yaml), every one commented:

| Parameter | Default | |
|---|---|---|
| `robot_ip`, `robot_port` | `10.21.31.103`, `30000` | the robot's UDP server (1.1.2) |
| `local_ip`, `local_port` | `0.0.0.0`, `0` | this end (any free port: the robot answers the heartbeat sender) |
| `heartbeat_hz` | 5.0 | the document: at least 1 Hz |
| `cmd_rate_hz`, `cmd_timeout_sec`, `stop_burst_sec` | 12.5, 0.5, 0.5 | velocity stream and watchdog |
| `auto_regular_mode`, `mode_refresh_sec` | true, 2.0 | |
| `max_vx`, `max_vy`, `max_wz` | 1.0, 0.5, 1.0 | `cmd_vel` clamp |
| `full_scale_vx`, `full_scale_vy`, `full_scale_wz` | 5.0, 2.0, 3.0 | speed at axis value 1: calibrate |
| `invert_vy`, `invert_wz` | false | sign fixes, [Verify on the real robot](../README.md#verify-on-the-real-robot) |
| `follow_direction` | true | negate X and Y while the forward direction is Back |
| `angles_in_degrees` | false | unit of the reported angles (switches to degrees by itself) |
| `odom_frame`, `base_frame`, `imu_frame`, `gps_frame` | `odom`, `base_link`, `imu_link`, `base_link` | message frames |
| `publish_tf` | false | `odom` → `base_link` from the dead reckoning |
| `state_hz`, `state_timeout_sec` | 2.0, 1.5 | no report for this long: disconnected |
| `response_timeout_sec` | 1.0 | wait for the call status response |
| `stand_timeout_sec`, `rl_timeout_sec`, `stand_up_gait` | 10.0, 5.0, `''` | `stand_up` |
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
the official Android client's framing (header, axis rounding); the link, the velocity policy and
the node run against `test/fake_robot.py` on the loopback interface. `test/smoke_test.sh` runs
the installed launch file against the simulator with the `ros2` command line (CI does both).
