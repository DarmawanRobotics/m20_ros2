# Changelog

All notable changes to m20_ros2. Format: [Keep a Changelog](https://keepachangelog.com),
versions follow [SemVer](https://semver.org).

## [Unreleased]

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
