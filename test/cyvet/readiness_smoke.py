#!/usr/bin/env python3
"""Attended test of the existing navigation readiness helper, then lie/release."""
import argparse
import os
import signal
import subprocess
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--readiness-script', required=True)
    parser.add_argument('--params', required=True)
    args = parser.parse_args()
    # Deliberately use the installed default selector, just like bringup.
    process = subprocess.Popen(['ros2', 'launch', 'nav_bridge', 'nav_bridge.launch.py',
                                'params_file:=' + args.params], start_new_session=True)
    try:
        subprocess.run(['python3', args.readiness_script, 'nav_bridge', '--topic', '/battery/level',
                        '--topic-timeout', '20', '--stand-timeout', '25'], check=True, timeout=50.)
        rclpy.init()
        node = rclpy.create_node('cyvet_readiness_cleanup')
        try:
            client = node.create_client(Trigger, '/nav_bridge_node/lie')
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('lie service unavailable')
            future = client.call_async(Trigger.Request())
            deadline = time.monotonic() + 20.
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
            if not future.done() or not future.result().success:
                raise RuntimeError('laying/release not confirmed')
            print('PASS: installed default selector, battery readiness, existing stand helper, lie/release',
                  flush=True)
        finally:
            node.destroy_node()
            rclpy.shutdown()
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=25.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
                raise RuntimeError('Launch cleanup timed out; check robot on site')


if __name__ == '__main__':
    main()
