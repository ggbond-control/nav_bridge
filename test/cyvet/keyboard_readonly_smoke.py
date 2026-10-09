#!/usr/bin/env python3
"""Validate the board's fresh zsh/keyboard DDS path without taking control.

Requires teleop_twist_keyboard and an otherwise stopped bridge.
Never calls stand/ready; every observed bridge status must remain read-only.
"""
import json
import os
import pty
import signal
import subprocess
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.create_node('cyvet_keyboard_readonly_check')
    status, commands, statuses = {}, [], []

    def receive(message):
        value = json.loads(message.data)
        status.clear()
        status.update(value)
        statuses.append(value)

    subscriptions = [
        node.create_subscription(String, '/nav_bridge_node/backend_status', receive, 10),
        node.create_subscription(Twist, '/cmd_vel', lambda m: commands.append(
            (m.linear.x, m.linear.y, m.angular.z)), 10),
    ]
    processes = []
    master, slave = pty.openpty()
    log = tempfile.TemporaryFile(mode='w+')

    def wait(predicate, timeout=20.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if any(s.get('control_owned') or s.get('navigation_ready') for s in statuses):
                raise RuntimeError('Bridge unexpectedly took control; stop this check')
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('Launch or keyboard exited early')
            if predicate():
                return
        raise RuntimeError('Condition timed out: ' + json.dumps(status))

    try:
        # Allow graph discovery; this check must never share an active bridge.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.05)
        if any(name == 'nav_bridge_node' for name, _ in node.get_node_names_and_namespaces()):
            raise RuntimeError('Another bridge is present; stop it before this check')
        if node.get_publishers_info_by_topic('/cmd_vel'):
            raise RuntimeError('Another velocity publisher is present')
        processes.append(subprocess.Popen(
            ['zsh', '-ic', 'ros2 launch nav_bridge nav_bridge.launch.py'],
            stdout=log, stderr=log, start_new_session=True))
        wait(lambda: status.get('connected') and status.get('battery_valid'))
        processes.append(subprocess.Popen(['zsh', '-ic',
            'ros2 run teleop_twist_keyboard teleop_twist_keyboard'],
            stdin=slave, stdout=slave, stderr=slave, start_new_session=True))
        wait(lambda: bool(node.get_publishers_info_by_topic('/cmd_vel')))
        checks = [('i', (.5, 0., 0.)), ('J', (0., .5, 0.)),
                  ('l', (0., 0., -1.)), ('k', (0., 0., 0.))]
        for key, expected in checks:
            start = len(commands)
            os.write(master, key.encode())
            wait(lambda: any(all(abs(a-b) < 1e-6 for a, b in zip(cmd, expected))
                             for cmd in commands[start:]), 5.)
        # Wait for fresh statuses after keyboard messages were consumed.
        count = len(statuses)
        wait(lambda: len(statuses) >= count+5)
        print(json.dumps({'success': True, 'domain': os.environ.get('ROS_DOMAIN_ID'),
            'dds_config': os.environ.get('CYCLONEDDS_URI'), 'keys': checks,
            'twists_received': commands, 'last_status': status,
            'controls_owned_observed': False}, ensure_ascii=False), flush=True)
    finally:
        # teleop reads raw stdin: type Ctrl-C instead of interrupting its blocked read.
        if len(processes) == 2 and processes[-1].poll() is None:
            os.write(master, b'\x03')
            try:
                processes[-1].wait(timeout=5.)
            except subprocess.TimeoutExpired:
                pass
        cleanup_failed = False
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=20.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    print('Process cleanup timed out', flush=True)
                    cleanup_failed = True
        log.seek(0)
        print(log.read()[-6000:], flush=True)
        log.close()
        os.close(master)
        os.close(slave)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        if cleanup_failed:
            raise RuntimeError('Process cleanup failed')


if __name__ == '__main__':
    main()
