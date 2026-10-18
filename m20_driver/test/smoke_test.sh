#!/usr/bin/env bash
# End-to-end smoke test: the installed driver (launch file + params file) against
# test/fake_robot.py on the loopback interface, checked with the ros2 command line.
# Topics, the state, stand_up, cmd_vel and its watchdog, a plain command, emergency_stop.
# Needs a sourced workspace (ROS 2 Foxy or later).
set -u
# ros2 output into a pipe is block-buffered and lost when timeout stops it
export PYTHONUNBUFFERED=1

share="$(ros2 pkg prefix m20_driver)/share/m20_driver"
work="$(mktemp -d)"
port=30000
ns=/m20
fail=0
pids=()

cleanup() {
  for p in "${pids[@]}"; do kill -INT "$p" 2>/dev/null; done
  sleep 2
  for p in "${pids[@]}"; do kill -9 "$p" 2>/dev/null; done
}
trap cleanup EXIT

check() {  # check <name> <pattern> <command...>: the command's output must match the pattern
  local name="$1" pattern="$2"
  shift 2
  local out
  out="$("$@" 2>&1)"
  if grep -qE -- "$pattern" <<<"$out"; then
    echo "ok   $name"
  else
    echo "FAIL $name"
    echo "$out" | tail -n 25
    fail=1
  fi
}

echo_for() {  # echo_for <seconds> <topic>: what ros2 topic echo prints in that time (Foxy: no --once)
  timeout "$1" ros2 topic echo "$2" 2>&1
}

# the installed parameters, pointed at the fake robot
sed -e "s/^\(\s*robot_ip:\).*/\1 127.0.0.1/" -e "s/^\(\s*robot_port:\).*/\1 $port/" \
  "$share/config/driver_params.yaml" > "$work/params.yaml"

python3 "$share/test/fake_robot.py" --port "$port" > "$work/robot.log" 2>&1 &
pids+=($!)
ros2 launch m20_driver driver.launch.py params_file:="$work/params.yaml" \
  > "$work/launch.log" 2>&1 &
pids+=($!)
for _ in $(seq 1 30); do
  ros2 topic list 2>/dev/null | grep -q "^$ns/state$" && break
  sleep 1
done
sleep 2

check "state: connected, sitting" "message: sit, basic, regular mode" echo_for 6 $ns/state
check "odom" "child_frame_id: base_link" echo_for 5 $ns/odom
check "imu" "frame_id: imu_link" echo_for 5 $ns/imu
check "joint_states" "hr_wheel_joint" echo_for 5 $ns/joint_states
check "battery" "percentage: 0.8" echo_for 6 $ns/battery
check "gps/fix" "latitude: -6.2" echo_for 6 $ns/gps/fix
check "diagnostics: robot errors" "motor temperature warning" echo_for 6 $ns/diagnostics

check "stand_up" "success=True, message='standing in RL control'" \
  timeout 45 ros2 service call $ns/stand_up std_srvs/srv/Trigger
check "lights_on" "success=True" timeout 30 ros2 service call $ns/lights_on std_srvs/srv/Trigger

# 0.5 m/s, 0.2 m/s, 0.3 rad/s over the default full scales (5, 2, 3) are ratios of 0.1; the
# fake robot runs 2 m/s at X = 1, so it reports 0.2 m/s
timeout 14 ros2 topic pub -r 10 $ns/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.2}, angular: {z: 0.3}}" > /dev/null 2>&1 &
publisher=$!
sleep 3
check "cmd_vel: axis ratios sent" "value: '?0.10 0.10 0.10'?" echo_for 4 $ns/state
check "cmd_vel: the robot moves" "x: 0.2" echo_for 4 $ns/odom
wait "$publisher"
sleep 2   # cmd_vel stopped: zeros after cmd_timeout_sec, then nothing
check "cmd_vel: watchdog zeros" "value: '?0.00 0.00 0.00'?" echo_for 4 $ns/state

check "emergency_stop" "success=True" \
  timeout 30 ros2 service call $ns/emergency_stop std_srvs/srv/Trigger
check "state: soft emergency stop" "soft_emergency_stop" echo_for 6 $ns/state

if grep -qE "Traceback|process has died" "$work/launch.log"; then
  echo "FAIL driver log"
  tail -n 40 "$work/launch.log"
  fail=1
else
  echo "ok   driver log clean"
fi
exit $fail
