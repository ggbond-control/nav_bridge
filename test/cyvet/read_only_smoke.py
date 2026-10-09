#!/usr/bin/env python3
"""Launch Cyvet briefly and verify telemetry. Does not call any motion service."""
import argparse
import json
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    launch_mode = parser.add_mutually_exclusive_group(required=True)
    launch_mode.add_argument('--executable')
    launch_mode.add_argument('--launch', action='store_true')
    parser.add_argument('--params', required=True)
    parser.add_argument('--seconds', type=float, default=10.)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('cyvet_readonly_smoke')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    counts, latest, subscriptions = {}, {}, []
    for topic, typ in [('/nav_bridge_node/backend_status', String), ('/battery/level', UInt8),
                       ('/imu/data', Imu), ('/joint_states', JointState), ('/leg_odom', Odometry)]:
        def receive(msg, topic=topic):
            counts[topic] = counts.get(topic, 0) + 1
            latest[topic] = msg
        subscriptions.append(node.create_subscription(typ, topic, receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
    log = tempfile.TemporaryFile(mode='w+')
    command = (['ros2', 'launch', 'nav_bridge', 'nav_bridge.launch.py', 'robot_type:=cyvet',
                'params_file:=' + args.params] if args.launch else
               [args.executable, '--ros-args', '--params-file', args.params])
    process = subprocess.Popen(command, env=os.environ.copy(), stdout=log, stderr=log, start_new_session=True)
    statuses = []
    try:
        deadline = time.monotonic() + 35.
        observation_end = None
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
            if process.poll() is not None:
                raise RuntimeError(f'Bridge exited {process.returncode}')
            message = latest.get('/nav_bridge_node/backend_status')
            if message:
                state = json.loads(message.data)
                statuses.append(state)
                if state['control_owned'] or state['navigation_ready']:
                    raise RuntimeError('Startup unexpectedly owns motion control')
                if state['connected'] and state['battery_valid'] and '/battery/level' in latest:
                    if observation_end is None:
                        observation_end = time.monotonic() + args.seconds
                    if time.monotonic() >= observation_end:
                        break
        else:
            raise RuntimeError('No fresh connected telemetry before timeout')
        imu = latest.get('/imu/data')
        result = {'success': True, 'counts': counts,
                  'battery_percent': latest['/battery/level'].data,
                  'last_status': statuses[-1], 'controls_owned_observed': False,
                  'imu_last': None if imu is None else {
                      'accel': [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z],
                      'gyro': [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z],
                      'quaternion': [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w],
                      'orientation_covariance_0': imu.orientation_covariance[0]}}
        print(json.dumps(result, ensure_ascii=False), flush=True)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.seek(0)
        print(log.read()[-4000:])
        log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    if process.returncode:
        raise SystemExit(process.returncode)


if __name__ == '__main__':
    main()
