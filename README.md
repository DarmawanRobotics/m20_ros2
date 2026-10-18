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
| [`m20_driver`](m20_driver/README.md) | the protocol driver: `cmd_vel` → axis commands in regular mode; reports → `odom`, `imu`, `joint_states`, `battery`, `gps/fix`, `state`, `diagnostics`; every robot command as a `std_srvs/Trigger` service; a simulated robot for trying it without one |
| [`m20_description`](m20_description/README.md) | URDF: Deep Robotics' model with its meshes (BSD-3-Clause), front/rear camera and LiDAR frames, `imu_link` |
| [`m20_bringup`](m20_bringup/README.md) | one launch for the driver and the URDF, RViz on request |

## Network

From the protocol document V1.3.0-0:

| | |
|---|---|
| robot | UDP server **10.21.31.103:30000** (the TCP server on 30001 is not used) |
| messages | 16-byte header `EB 91 EB 90`, ASDU length and message ID (little endian), `0x01` = JSON; then `{"PatrolDevice": {"Type", "Command", "Time", "Items"}}` |
| heartbeat | Type 100 / Command 100, at least 1 Hz (the driver sends 5 Hz) |
| reports | to whoever sends the heartbeat: basic status 2 Hz, motion status 10 Hz, device status 2 Hz, abnormal status 2 Hz |
| answers | a call status response (ErrorCode, ErrorMessage) to every request |
| velocity | axis command Type 2 / Command 21, ratios in [-1, 1], **regular usage mode only**, moves the robot in RL control; from one client at a time |

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

Then work through [Verify on the real robot](#verify-on-the-real-robot), including the speed
calibration and the `cmd_vel` signs (steps 5 to 7), before any autonomy drives the robot.

## Interface

Everything under `/m20` (launch argument `namespace`).

| | Name | Type | |
|---|---|---|---|
| sub | `cmd_vel` | `geometry_msgs/Twist` | REP-103 in `base_link`; zeros 0.5 s after the last command, then nothing |
| pub | `odom` | `nav_msgs/Odometry` | `odom` → `base_link`, dead-reckoned from the reported velocities and heading; no TF unless `publish_tf` |
| pub | `imu` | `sensor_msgs/Imu` | attitude, rates (and acceleration on newer firmware), frame `imu_link` |
| pub | `joint_states` | `sensor_msgs/JointState` | 12 leg angles and 4 wheels, named as in the URDF |
| pub | `battery` | `sensor_msgs/BatteryState` | both batteries, `percentage` 0–1 |
| pub | `gps/fix` | `sensor_msgs/NavSatFix` | the robot's GPS |
| pub | `state` | `diagnostic_msgs/DiagnosticStatus` | connected, motion state, gait, usage mode, charge, e-stop, sleep, battery ... as key/values |
| pub | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | driver and link, robot errors (the document's error table), temperatures, power |

**Services** (`std_srvs/Trigger`; each waits for the robot's answer, a refusal fails the call
with the robot's error code):

| | |
|---|---|
| posture | `stand_up` (Stand, then RL control), `sit_down`, `idle`, `damping`, `rl_control`, `emergency_stop` (soft) |
| gaits | `gait_basic`, `gait_stair` (standard), `gait_flat_agile`, `gait_stair_agile` (agile) |
| usage modes | `regular_mode`, `navigation_mode`, `assist_mode` |
| other | `lights_on`, `lights_off`, `charge_start`, `charge_stop`, `sleep`, `wake`, `sync_time` |

## Usage

```bash
# stand up and walk
ros2 service call /m20/stand_up std_srvs/srv/Trigger
ros2 service call /m20/gait_flat_agile std_srvs/srv/Trigger      # agile gait: for autonomy
ros2 topic pub -r 10 /m20/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# stop, sit, lights, docking
ros2 service call /m20/emergency_stop std_srvs/srv/Trigger
ros2 service call /m20/sit_down std_srvs/srv/Trigger
ros2 service call /m20/lights_on std_srvs/srv/Trigger
ros2 service call /m20/charge_start std_srvs/srv/Trigger

# state, errors, temperatures, speed estimates
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
2. **Angle unit:** turn the robot 90° by remote: the yaw of `/m20/imu` (and `/m20/odom`) must
   change by about 1.57 rad. `state.angle_unit` reads `deg` if the robot sends degrees (the
   driver switches by itself); set `angles_in_degrees` to match so it holds from the first report.
3. **Stand:** `ros2 service call /m20/stand_up std_srvs/srv/Trigger` answers `standing in RL
   control`. Note from which states the robot accepts Stand.
4. **Regular mode:** with `auto_regular_mode: true` (default) `state.usage_mode` turns `regular`
   when `cmd_vel` starts; close the official app's control first (one client: `0xE006`).
5. **Forward:** `ros2 topic pub -r 10 /m20/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}"`
   moves it forward, slowly (uncalibrated). Stop the publisher: it stops within 0.5 s.
6. **Calibrate speeds:** drive steadily (e.g. `x: 0.5`, then `y: 0.2`, then `angular: {z: 0.5}`)
   and read `full_scale_vx_estimate`, `_vy_`, `_wz_` in `/m20/diagnostics` (reported speed over
   sent ratio). Put them into `full_scale_vx`, `full_scale_vy`, `full_scale_wz` of your
   parameters; `odom`'s twist then matches `cmd_vel`.
7. **Turn and strafe (REP-103):** `{angular: {z: 0.3}}` must turn the robot **left**
   (counter-clockwise seen from above), else set `invert_wz: true`; `{linear: {y: 0.1}}` must
   move it **left**, else `invert_vy: true`. If you set the robot's forward direction to Back
   (`state.direction`), check that `x: 0.2` still moves it towards its front (the driver negates
   X and Y then, `follow_direction`) and that `odom` still counts forward.
8. **Joints and frames:** `ros2 launch m20_bringup bringup.launch.py rviz:=true`: the legs of the
   model move like the real ones (joint signs against the URDF axes) and the wheels turn the
   right way. The model gives the rear camera and rear LiDAR no rotation: check their frames
   against their data.
9. **Batteries and GPS:** `battery.location` lists `rear, front` (the official client's order of
   `BatteryList`); compare the percentages with the app. Outdoors, `gps/fix` shows a fix
   (`FixQuality` read as NMEA: 1 GPS, 2 differential, 4/5 RTK).
10. **Sleep and charging:** `sleep` / `wake` keep the robot's own auto-sleep settings (read with
   1101/7); `charge_start` near the dock, `charge_stop` after.

## Safety

- The driver stops on its own: no `cmd_vel` for 0.5 s gives zeros, then silence. Only one node
  (your control mux) should publish on `/m20/cmd_vel`.
- `emergency_stop` zeroes the velocity and sends the soft emergency stop; the remote and the
  robot's hard emergency stop always work too.
- Until the speeds are calibrated the robot runs slower than `cmd_vel` asks (high `full_scale_*`
  defaults); `max_vx`, `max_vy`, `max_wz` clamp every command.
- `sit_down`, `idle` and `damping` are refused while `cmd_vel` moves the robot. Idle and damping
  change how the joints hold: use them with the robot sitting.
- With `auto_regular_mode` the driver switches the robot to regular mode when motion starts: do
  not run it while the robot's own navigation is driving.
- `charge_start` makes the robot dock by itself.

## Develop

```bash
colcon build && colcon test --executor sequential && colcon test-result --verbose
```

The codec is tested against the document's header layout and JSON examples and the official
Android client's framing; the UDP link, the velocity policy and the node run against a simulated
M20 on the loopback interface (`m20_driver/test/fake_robot.py`); the URDF against the official
model's numbers and the driver's joint names. CI runs the same in a `ros:foxy` container, plus
an end-to-end smoke test of the installed launch file against the simulator, `check_urdf` and
shellcheck.

The URDF and its meshes are Deep Robotics' model
([DeepRoboticsLab/deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model),
BSD-3-Clause, [license](m20_description/LICENSE.deep_robotics_model)). m20_ros2 is released
under the [Apache License 2.0](LICENSE).
