#!/usr/bin/env python3
"""
End-to-end checks of a running m20_driver against test/fake_robot.py (run by smoke_test.sh).

One ROS node for all of it: short-lived command line tools killed by a timeout leave stale
DDS state behind on Foxy, which made checks after the first few fail at random.

    python3 smoke_check.py --namespace /m20

Prints 'ok   <check>' or 'FAIL <check>' per check; exit status 1 when any check failed.
"""

import argparse
import math
import sys
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import BatteryState, Imu, JointState, NavSatFix
from std_srvs.srv import Trigger

TOPICS = {
    'odom': Odometry,
    'imu': Imu,
    'joint_states': JointState,
    'battery': BatteryState,
    'gps/fix': NavSatFix,
    'state': DiagnosticStatus,
    'diagnostics': DiagnosticArray,
}


class Checker:
    """Subscribes to every driver topic and runs the checks in order."""

    def __init__(self, namespace: str):
        """Create the node and spin it on its own thread."""
        self.node = rclpy.create_node('m20_smoke_check', namespace=namespace)
        self.lock = threading.Lock()
        self.latest = {}
        for name, kind in TOPICS.items():
            self.node.create_subscription(kind, name, self.keeper(name), 10)
        self.cmd = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.failed = 0
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def keeper(self, name: str):
        """Return a callback that keeps the latest message of a topic."""
        def keep(msg):
            with self.lock:
                self.latest[name] = msg
        return keep

    def message(self, name: str):
        """Return the latest message of a topic, None before the first."""
        with self.lock:
            return self.latest.get(name)

    def state(self) -> dict:
        """Return the key/values of the latest state."""
        msg = self.message('state')
        return {kv.key: kv.value for kv in msg.values} if msg is not None else {}

    @staticmethod
    def wait(predicate, timeout: float) -> bool:
        """Poll until the predicate holds."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return True
            time.sleep(0.05)
        return bool(predicate())

    def check(self, name: str, predicate, timeout: float = 15.0, detail=None):
        """Print ok or FAIL for one check."""
        if self.wait(predicate, timeout):
            print(f'ok   {name}')
            return
        self.failed += 1
        print(f'FAIL {name}')
        if detail is not None:
            print(f'     {detail()}')

    def call(self, name: str, timeout: float = 30.0):
        """Call a Trigger service of the driver; None when it does not answer."""
        client = self.node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(Trigger.Request())
        if not self.wait(future.done, timeout):
            return None
        return future.result()

    def check_call(self, name: str, expected_message: str = None):
        """Call a service and check its answer."""
        answer = self.call(name)
        ok = answer is not None and answer.success and (
            expected_message is None or answer.message == expected_message)
        self.check(f'{name}', lambda: ok, timeout=0.0,
                   detail=lambda: 'no answer' if answer is None else answer)

    def drive(self, twist: Twist, seconds: float):
        """Publish a velocity at 10 Hz for a while."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd.publish(twist)
            time.sleep(0.1)

    def run(self) -> int:
        """Run every check; return the number of failures."""
        latest = self.message
        self.check('state: connected, sitting',
                   lambda: (latest('state') is not None
                            and latest('state').message.startswith('sit, basic, regular mode')),
                   timeout=30.0, detail=lambda: latest('state'))
        self.check('odom', lambda: latest('odom') is not None
                   and latest('odom').child_frame_id == 'base_link')
        self.check('imu', lambda: latest('imu') is not None
                   and latest('imu').header.frame_id == 'imu_link')
        self.check('joint_states', lambda: latest('joint_states') is not None
                   and 'hr_wheel_joint' in latest('joint_states').name)
        self.check('battery', lambda: latest('battery') is not None
                   and abs(latest('battery').percentage - 0.85) < 1e-3)
        self.check('gps/fix', lambda: latest('gps/fix') is not None
                   and abs(latest('gps/fix').latitude + 6.2) < 1e-6)
        self.check('diagnostics: robot errors', lambda: latest('diagnostics') is not None and any(
            'motor temperature warning' in s.message for s in latest('diagnostics').status))

        self.check_call('stand_up', 'standing in RL control')
        self.check_call('lights_on')
        self.check('state: lights on', lambda: self.state().get('lights') == 'front 1, back 1')

        # 0.5 m/s, 0.2 m/s, 0.3 rad/s over the default full scales (5, 2, 3) are ratios of 0.1;
        # the fake robot runs 2 m/s at X = 1, so it reports 0.2 m/s
        twist = Twist()
        twist.linear.x, twist.linear.y, twist.angular.z = 0.5, 0.2, 0.3
        moving = threading.Thread(target=self.drive, args=(twist, 4.0))
        moving.start()
        self.check('cmd_vel: axis ratios sent',
                   lambda: self.state().get('cmd_axis') == '0.10 0.10 0.10',
                   timeout=4.0, detail=self.state)
        self.check('cmd_vel: the robot moves',
                   lambda: math.isclose(latest('odom').twist.twist.linear.x, 0.2, abs_tol=1e-6),
                   timeout=4.0, detail=lambda: latest('odom').twist.twist)
        moving.join()
        self.check('cmd_vel: watchdog zeros',
                   lambda: self.state().get('cmd_axis') == '0.00 0.00 0.00',
                   timeout=5.0, detail=self.state)

        self.check_call('emergency_stop')
        self.check('state: soft emergency stop',
                   lambda: self.state().get('motion_state') == 'soft_emergency_stop',
                   detail=self.state)
        return self.failed

    def close(self):
        """Stop spinning and destroy the node."""
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()


def main():
    """Run the checks once."""
    parser = argparse.ArgumentParser(description=__doc__.strip().split('\n\n')[0])
    parser.add_argument('--namespace', default='/m20')
    args = parser.parse_args()
    rclpy.init()
    checker = Checker(args.namespace)
    try:
        failed = checker.run()
    finally:
        checker.close()
        rclpy.shutdown()
    sys.exit(1 if failed else 0)


if __name__ == '__main__':
    main()
