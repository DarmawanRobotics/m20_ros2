# m20_bringup

The Lynx M20 in one launch: `m20_driver` and `robot_state_publisher` with the URDF, both in the
robot namespace (the driver's `joint_states` drive the model), RViz on request.

## Usage

```bash
ros2 launch m20_bringup bringup.launch.py                        # namespace m20, 10.21.31.103
ros2 launch m20_bringup bringup.launch.py rviz:=true
ros2 launch m20_bringup bringup.launch.py driver_params:=$HOME/m20_params.yaml
ros2 launch m20_bringup bringup.launch.py robot_ip:=127.0.0.1    # against the fake robot
```

## Launch

`launch/bringup.launch.py`:

| Argument | Default | |
|---|---|---|
| `namespace` | `m20` | driver topics and services, `robot_state_publisher` |
| `robot_ip` | empty | the robot's address; empty: `driver_params` (`10.21.31.103`) |
| `driver_params` | [`m20_driver/config/driver_params.yaml`](../m20_driver/config/driver_params.yaml) | a copy with this robot's settings (calibrated `full_scale.<gait>.*`, `angle_unit`, sign fixes, frames) |
| `rviz` | `false` | RViz with the robot model |

## Nodes

| Node | Namespace | |
|---|---|---|
| `m20_driver` | `m20` | `cmd_vel` in (through the velocity gate); `odom`, `imu`, `joint_states`, `battery`, `gps/fix`, `state`, `diagnostics`; confirmed Trigger services ([m20_driver](../m20_driver/README.md)) |
| `robot_state_publisher` | `m20` | `robot_description`, TF `base_link` → legs, wheels, `imu_link`, cameras, LiDARs ([m20_description](../m20_description/README.md)) |
| `rviz2` (`rviz:=true`) | root | the model and TF |

The parameter file uses a `/**:` key: a bare node name would not match a node in a namespace.
Nothing publishes `odom` → `base_link` TF unless the driver's `publish_tf` is set (leave it to
a localization stack otherwise).
