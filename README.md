<div align="center">

# m20_ros2

**ROS 2 Foxy robot package for the Deep Robotics Lynx M20: the robot's inspection protocol as a
driver with standard messages only, the official URDF with its camera and LiDAR frames, and one
launch for both.**

[![CI](https://github.com/DarmawanRobotics/m20_ros2/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/DarmawanRobotics/m20_ros2/actions/workflows/ci.yml)
[![ROS 2](https://img.shields.io/badge/ROS%202-Foxy-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/foxy/)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-E95420?logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/20.04/)
![Python](https://img.shields.io/badge/Python-3.8-3776AB?logo=python&logoColor=white)
[![License](https://img.shields.io/badge/license-Apache%202.0-blue)](LICENSE)

[Packages](#packages) · [Network](#network) · [Quick start](#quick-start) ·
[Interface](#interface) · [Usage](#usage) · [Verify on the real robot](#verify-on-the-real-robot) ·
[Safety](#safety) · [Develop](#develop)

</div>

---

The Lynx M20 takes requests and reports its state over UDP with JSON messages (Deep Robotics
*Inspection Communication Protocol* V1.3.0-0). This repository turns that into ROS 2 Foxy topics
and services on any computer that reaches the robot: velocity, odometry, IMU, joints, battery,
GPS, errors, and every motion state, gait, mode, light, charging and sleep command. The robot's
own ROS 2 topics (custom `drdds` types, on its hosts only) are not needed.

**Docs:** [driver](m20_driver/README.md) · [URDF and frames](m20_description/README.md) ·
[bringup](m20_bringup/README.md) · [changelog](CHANGELOG.md)

## Packages

| Package | |
|---|---|
| [`m20_driver`](m20_driver/README.md) | the protocol driver: `cmd_vel` through a velocity gate (holds, takeover latch, E006, watchdog) → axis commands in regular mode; reports → `odom`, `imu`, `joint_states`, `battery`, `gps/fix`, `state`, `diagnostics`; every robot command as a `std_srvs/Trigger` service that succeeds only when the robot confirmed it; a simulated robot for trying it without one |
| [`m20_description`](m20_description/README.md) | URDF: Deep Robotics' model with its meshes (BSD-3-Clause), front/rear camera and LiDAR frames, `imu_link` |
| [`m20_bringup`](m20_bringup/README.md) | one launch for the driver and the URDF, RViz on request |

## Network

From the protocol document V1.3.0-0:

| | |
|---|---|
| robot | UDP server **10.21.31.103:30000** on the wired payload network (the TCP server on 30001 is not used); the official remote app talks to `10.21.33.103` (SDK `IpConfig`) |
| messages | 16-byte header `EB 91 EB 90`, ASDU length and message ID (little endian), `0x01` = JSON; then `{"PatrolDevice": {"Type", "Command", "Time", "Items"}}` |
| heartbeat | Type 100 / Command 100, at least 1 Hz (the driver sends 5 Hz) |
| reports | to whoever sends the heartbeat: basic status 2 Hz, motion status 10 Hz, device status 2 Hz, abnormal status 2 Hz |
| answers | a call status response (ErrorCode, ErrorMessage) to every request |
| velocity | axis command Type 2 / Command 21, ratios in [-1, 1], 20 Hz recommended, **regular usage mode only**, moves the robot in RL control; from **one client within 2 s** (E006) |

Nothing has to be configured on the robot: it reports to the address and port the heartbeats come
from. This computer only needs a route to `10.21.31.103` (`ping 10.21.31.103`).

## Quick start

Ubuntu 20.04, ROS 2 Foxy.

```bash
# 1. Sources and dependencies (Foxy is end-of-life: rosdep needs --include-eol-distros)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/DarmawanRobotics/m20_ros2.git
cd ~/ros2_ws
rosdep update --rosdistro foxy --include-eol-distros
rosdep install --from-paths src --ignore-src -y --rosdistro foxy

# 2. Build
colcon build && source install/setup.bash

# 3. Launch: the driver and the URDF in the namespace m20
ros2 launch m20_bringup bringup.launch.py

# 4. In a second terminal
ros2 topic echo /m20/state
ros2 service call /m20/stand_up std_srvs/srv/Trigger
```

No robot at hand: start the simulated M20 and point the driver at it.

```bash
python3 $(ros2 pkg prefix m20_driver)/share/m20_driver/test/fake_robot.py --port 30000
ros2 launch m20_bringup bringup.launch.py robot_ip:=127.0.0.1 rviz:=true
```

Then work through [Verify on the real robot](#verify-on-the-real-robot), including the
emergency stop, the takeover, the speed calibration and the `cmd_vel` signs (steps 4 to 10),
before any autonomy drives the robot.

## Interface

Everything under `/m20` (launch argument `namespace`).

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | REP-103 in `base_link`; sent only while armed and moving, then zeros for 0.5 s, then nothing (also while zero Twists keep coming); held by services, a takeover or E006 ([velocity gate](m20_driver/README.md#velocity)) |
| pub | `odom` | `nav_msgs/Odometry` | `odom` → `base_link`, dead-reckoned from the reported velocities and heading; no TF unless `publish_tf` |
| pub | `imu` | `sensor_msgs/Imu` | attitude once the angle unit is known, rates (and acceleration on newer firmware), frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | 12 leg angles and 4 wheels, named as in the URDF |
| pub | `battery` | `sensor_msgs/BatteryState` | both batteries, `percentage` 0–1 |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` | the robot's GPS |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | connected, motion state, `velocity` (driving, idle, hold, disarmed, takeover, axis_owner_other), gait, usage mode, charge, e-stop, sleep, battery ... as key/values |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | driver and link, robot errors (the document's error table), temperatures, power |

**Services** (`std_srvs/Trigger`): success only when the robot confirmed the command (its call
status response, and for motion states the reported state); no answer is `unconfirmed`, a refusal
fails with the robot's error code.

| | |
|---|---|
| posture | `stand_up` (Stand, then RL control), `sit_down`, `idle`, `damping`, `rl_control` |
| stop | `emergency_stop` (soft, confirmed or an ERROR), `release_emergency_stop` (from the SDK, not in V1.3.0) |
| gaits | `gait_basic`, `gait_stair` (standard), `gait_flat_agile`, `gait_stair_agile` (agile); in RL control and regular mode |
| usage modes | `regular_mode`, `navigation_mode`, `assist_mode` |
| other | `lights_on`, `lights_off`, `charge_start`, `charge_stop`, `charge_cancel` (from the SDK), `sleep`, `wake`, `sync_time` |

## Usage

```bash
# stand up and walk
ros2 service call /m20/stand_up std_srvs/srv/Trigger
ros2 service call /m20/gait_flat_agile std_srvs/srv/Trigger      # agile gait: for autonomy
ros2 topic pub -r 10 /m20/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# stop (cmd_vel stays held until stand_up or rl_control), sit, lights, docking
ros2 service call /m20/emergency_stop std_srvs/srv/Trigger
ros2 service call /m20/release_emergency_stop std_srvs/srv/Trigger
ros2 service call /m20/sit_down std_srvs/srv/Trigger
ros2 service call /m20/lights_on std_srvs/srv/Trigger
ros2 service call /m20/charge_start std_srvs/srv/Trigger

# state (velocity: why cmd_vel does or does not drive), errors, temperatures, speed estimates
ros2 topic echo /m20/state
ros2 topic echo /m20/diagnostics
```

Your own settings go into a copy of
[driver_params.yaml](m20_driver/config/driver_params.yaml):
`ros2 launch m20_bringup bringup.launch.py driver_params:=$HOME/m20_params.yaml`.

## Verify on the real robot

The protocol document leaves some things open; the driver takes the safe or the documented
choice and these steps settle them. On a new robot, in an open area, with someone at the
remote and the hard emergency stop:

1. **Reports:** `ros2 topic hz /m20/odom` shows ~10 Hz and `/m20/state` says `connected: true`
   with the real motion state and gait.
2. **Angle unit:** drive a turn faster than 0.3 rad/s: `state.angle_unit` becomes `rad (auto)` or
   `deg (auto)` (the document says rad, the official client degrees). Turn 90° by remote: the yaw
   of `/m20/imu` changes by about 1.57 rad. Then set `angle_unit` to that unit.
3. **Stand:** `ros2 service call /m20/stand_up std_srvs/srv/Trigger` answers `standing in RL
   control`. Note from which states the robot accepts Stand.
4. **Emergency stop:** while driving, `emergency_stop` answers `soft emergency stop confirmed` and
   the robot stops. Then find out what releases it: `release_emergency_stop` (MotionParam −2,
   from the SDK) or `stand_up` straight from state 2 (the SDK allows Stand there). `cmd_vel`
   stays held until `stand_up` or `rl_control`, and then needs a zero first.
5. **Regular mode and takeover:** with `auto_regular_mode` (default) a robot that was in
   navigation mode since boot is switched to regular when `cmd_vel` starts. While streaming,
   switch to navigation in the app: the driver stops, `state.velocity` says `takeover
   (navigation mode)` and it never switches back by itself. Hand back to regular in the app (or
   call `regular_mode`): it drives again after `cmd_vel` was zero once.
6. **One axis client:** while the driver streams, the remote's joystick is locked out (E006, 2 s
   rule); after the stop burst the driver is quiet and the remote works again. Use the remote's
   joystick while `cmd_vel` streams: `state.velocity` shows `axis_owner_other` and the driver
   yields until `cmd_vel` pauses.
7. **Forward:** `ros2 topic pub -r 10 /m20/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
   moves it forward, slowly (uncalibrated). Stop the publisher: it stops within 0.5 s.
8. **Calibrate speeds per gait:** in each gait, drive steadily (e.g. `x: 0.5`, then `y: 0.2`,
   then `angular: {z: 0.5}`) and read `full_scale.<gait>.vx_estimate` (`vy`, `wz`) in
   `/m20/diagnostics`. Put them into `full_scale.<gait>.vx`, `.vy`, `.wz` of your parameters;
   `odom`'s twist then matches `cmd_vel`. A gait far faster than its full scale trips the
   overspeed guard (`disarmed:overspeed`).
9. **Turn and strafe (REP-103):** `{angular: {z: 0.3}}` must turn the robot **left**
   (counter-clockwise seen from above), else set `invert_wz: true`; `{linear: {y: 0.1}}` must
   move it **left**, else `invert_vy: true`.
10. **Direction Back:** set the robot's forward direction to Back (`state.direction`): `x: 0.2`
    must still move it towards its front and `y: 0.1` to its left (the driver negates X and Y,
    `follow_direction`), and `odom`'s twist must still read positive x. If it reads negative,
    set `direction_flips_reports: true` (the diagnostics warn about it after 2 s).
11. **Joints and frames:** `ros2 launch m20_bringup bringup.launch.py rviz:=true`: the legs of
    the model move like the real ones (joint signs against the URDF axes) and the wheels turn
    the right way. The model gives the rear camera and rear LiDAR no rotation: check their frames
    against their data.
12. **Batteries and GPS:** `battery.location` lists `rear, front` (the official client's order of
    `BatteryList`); compare the percentages with the app. Outdoors, `gps/fix` shows a fix
    (`FixQuality` read as NMEA) and an altitude: find out whether it is above sea level or the
    ellipsoid.
13. **Sleep and charging:** `sleep` / `wake` keep the robot's own auto-sleep settings (read with
    1101/7 every time); `charge_start` near the dock; `charge_cancel` while it goes to the dock,
    `charge_stop` while charging or on the dock (the SDK's meaning of Charge 0).
14. **Lost controller:** stop the driver while it streams: three zeros go out. Its heartbeat also
    pauses when the driver hangs: check what the robot does then (the SDK knows error 0x8309,
    controller heartbeat timeout).

## Safety

- The driver stops on its own: no `cmd_vel` for 0.5 s gives zeros for 0.5 s, then silence, and a
  stream of zero Twists sends nothing. Only one node (your control mux) should publish on
  `/m20/cmd_vel`.
- `emergency_stop` holds `cmd_vel` first, sends a zero and the soft emergency stop three times,
  and succeeds only when the robot confirms it; otherwise it fails with `soft e-stop NOT
  confirmed` and logs an ERROR. It aborts any service that is still waiting. The remote and the
  robot's hard emergency stop always work too.
- `emergency_stop`, `sit_down`, `idle`, `damping`, `sleep`, `charge_start`, `navigation_mode` and
  `assist_mode` hold `cmd_vel`; only `stand_up` / `rl_control` (and `regular_mode` for the mode
  holds) lift it, and a running stream then has to be zero once: a stale stream never drives off.
- A takeover (the robot leaves regular mode after it was in regular mode: the app, the remote,
  the robot) latches: no axis command and no mode request until the robot reports regular mode
  again. The driver never switches a robot back that someone else switched away.
- While the driver streams, the remote's joystick is locked out (the robot takes axis commands
  from one client within 2 s); after the stop burst the driver is quiet. When another client
  holds the axis channel (E006) the driver yields until `cmd_vel` pauses.
- No service reports success without the robot's answer (and for motion states the reported
  state).
- Until the speeds are calibrated the robot runs slower than `cmd_vel` asks (high
  `full_scale.<gait>.*` defaults); `max_vx`, `max_vy`, `max_wz` clamp every command, and a
  reported speed far above them stops the stream (overspeed).
- `sit_down`, `idle` and `damping` are refused while the robot walks; `idle` and `damping` only
  from a sitting state. `stand_up` is refused on a hard emergency stop or while charging.
- Invalid parameters stop the driver before it contacts the robot.
- `charge_start` makes the robot dock by itself.

## Develop

```bash
colcon build && colcon test --executor sequential && colcon test-result --verbose
```

The codec is tested against the document's header layout and JSON examples and the official
Android client's framing; every rule of the velocity gate has a unit test; the UDP link and the
node run against a simulated M20 on the loopback interface (`m20_driver/test/fake_robot.py`),
which can also answer with wrong message IDs, stay silent, refuse with E006, change its usage
mode on its own and report degrees; the URDF against the official model's numbers and the
driver's joint names. CI runs the same in a `ros:foxy` container, plus
an end-to-end smoke test of the installed launch file against the simulator, `check_urdf` and
shellcheck.

The URDF and its meshes are Deep Robotics' model
([DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model),
BSD-3-Clause, [license](m20_description/LICENSE.deep_robotics_model)). m20_ros2 is released
under the [Apache License 2.0](LICENSE).
