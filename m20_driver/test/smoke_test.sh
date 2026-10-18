#!/usr/bin/env bash
# End-to-end smoke test: the installed driver (launch file + params file) against
# test/fake_robot.py on the loopback interface. The ros2 command line checks the graph and one
# service call; test/smoke_check.py (one ROS node) checks topics, state, stand_up, cmd_vel and
# its watchdog, a plain command and emergency_stop.
# Needs a sourced workspace (ROS 2 Foxy or later).
set -u
# python output into a pipe is block-buffered and lost when the process is stopped
export PYTHONUNBUFFERED=1

here="$(cd "$(dirname "$0")" && pwd)"
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

# the graph: every topic and service of the driver (the command line exits by itself)
topics="$(ros2 topic list 2>&1)"
services="$(ros2 service list 2>&1)"
for topic in cmd_vel odom imu joint_states battery gps/fix state diagnostics; do
  check "topic $ns/$topic" "^$ns/$topic\$" echo "$topics"
done
missing=""
for service in stand_up sit_down idle damping rl_control emergency_stop gait_basic gait_stair \
    gait_flat_agile gait_stair_agile regular_mode navigation_mode assist_mode lights_on \
    lights_off charge_start charge_stop sleep wake sync_time; do
  grep -q "^$ns/$service\$" <<<"$services" || missing="$missing $service"
done
check "services: all 20" "^none missing$" echo "${missing:-none missing}"

timeout -s INT -k 10 120 python3 "$here/smoke_check.py" --namespace "$ns" || fail=1

check "ros2 service call lights_off" "success=True" \
  timeout -s INT -k 5 45 ros2 service call $ns/lights_off std_srvs/srv/Trigger

if grep -qE "Traceback|process has died" "$work/launch.log"; then
  echo "FAIL driver log"
  tail -n 40 "$work/launch.log"
  fail=1
else
  echo "ok   driver log clean"
fi
exit $fail
