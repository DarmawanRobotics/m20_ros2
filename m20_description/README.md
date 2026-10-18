# m20_description

URDF of the Deep Robotics Lynx M20: Deep Robotics' own model with its meshes, camera, LiDAR and
IMU frames, as xacro.

## Usage

```bash
ros2 launch m20_description display.launch.py                  # RViz with joint sliders
ros2 launch m20_description description.launch.py namespace:=m20
```

## Launch

| File | | Arguments |
|---|---|---|
| `description.launch.py` | `robot_state_publisher` with the URDF | `namespace` (`m20`) |
| `display.launch.py` | the above, joint sliders (`joint_state_publisher_gui`) and RViz | `namespace`, `gui` (`true`; `false`: `joint_state_publisher` without sliders) |

In the namespace, `robot_state_publisher` turns the driver's `joint_states` into TF;
[m20_bringup](../m20_bringup/README.md) starts both.

## Model

Links, joints, limits, meshes, collisions and inertials are those of
[deep_robotics_model](https://github.com/DeepRoboticsLab/deep_robotics_model)
`M20/urdf/M20.urdf` (BSD-3-Clause, [LICENSE.deep_robotics_model](LICENSE.deep_robotics_model)),
with every number unchanged and the official names: `base_link`, `{fl,fr,hl,hr}_{hipx,hipy,knee,wheel}`,
revolute `*_hipx_joint`, `*_hipy_joint`, `*_knee_joint` and continuous `*_wheel_joint`, the
names `m20_driver` publishes. Changed from the official file: the robot is called `m20`, the
MuJoCo compiler block is gone, meshes are `package://m20_description/meshes/...`, the unnamed
grey materials share one name, and `imu_link` is added.

## Frames (relative to `base_link`, metres)

`base_link` is the official root: the centre of the body, x forward, y left, z up. The legs on
the battery compartment side are the left ones, the hard emergency stop is at the back.

| Frame | Position | Source |
|---|---|---|
| `*_hipx_joint` | (±0.3141, ±0.0685, 0) | model |
| `*_hipy_joint` | (0, 0, 0) from HipX | model |
| `*_knee_joint` | (0, ±0.0984, −0.25) from HipY | model |
| `*_wheel_joint` | (0, ±0.059676, −0.25) from the knee | model |
| `front_camera_link`, `rear_camera_link` | (±0.37646, 0, 0.03738) | model |
| `front_lidar_link`, `rear_lidar_link` | (±0.32028, 0, −0.013) | model |
| `imu_link` | (0, 0, 0) | the document gives no IMU position: identity, so `imu` reads as the body's attitude |

Axes: HipX −x, HipY, knee and wheel −y; limits HipX −0.436…0.611 (left) / −0.611…0.436 (right),
HipY −2.583…2.286 (front) / −2.286…2.583 (hind), knee −2.792…2.809 (front) / −2.809…2.792
(hind) rad. The model gives the rear camera and LiDAR no rotation: they look forward in the
URDF, check that against their data ([Verify on the real robot](../README.md#verify-on-the-real-robot)).

`base_link` carries the body's inertia, so `robot_state_publisher` warns that KDL ignores a root
inertia; harmless here (nothing is simulated).

## Test

```bash
colcon test --packages-select m20_description && colcon test-result --verbose
```

The test expands the xacro, checks that the moving joints are `m20_driver`'s `JOINT_NAMES` (and
its wheel indices the continuous joints), the legs and sensor frames against the official
numbers, that every mesh is in the package, and the tree. CI also runs `check_urdf`.
