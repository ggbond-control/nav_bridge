#!/usr/bin/env python3
"""Run the real X30 node against loopback UDP; never connects to hardware."""
import argparse
import os
import signal
import socket
import struct
import subprocess
import tempfile
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '216'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'

import rclpy
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    udp.settimeout(.1)
    state = None
    packets = []
    stopping = threading.Event()

    def respond():
        while not stopping.is_set():
            try:
                data, peer = udp.recvfrom(4096)
            except socket.timeout:
                continue
            code, value, kind = struct.unpack('<III', data[:12])
            packets.append((code, value, time.monotonic()))
            if state is not None and code in (0x91910250, 0x91910253):
                udp.sendto(struct.pack('<III', code, state, 0), peer)

    thread = threading.Thread(target=respond)
    thread.start()
    rclpy.init()
    node = rclpy.create_node('x30_charge_timeout_test')
    charge = node.create_client(SetParameters, '/nav_bridge_node/charge_command')
    parameters = node.create_client(GetParameters, '/nav_bridge_node/get_parameters')
    process = None
    with tempfile.TemporaryFile(mode='w+') as log:
        try:
            process = subprocess.Popen([
                args.executable, '--ros-args',
                '-p', 'motion_host_ip:=127.0.0.1', '-p', f'motion_host_port:={udp.getsockname()[1]}',
                '-p', 'local_recv_port:=0', '-p', 'charge_host_ip:=127.0.0.1',
                '-p', f'charge_host_port:={udp.getsockname()[1]}', '-p', 'charge_local_port:=0',
                '-p', f'charge_config_port:={udp.getsockname()[1]}',
                '-p', 'enable_charge_state_query:=false', '-p', 'startup_acquire_control:=false',
                '-p', 'charge_command_timeout_ms:=800',
            ], stdout=log, stderr=subprocess.STDOUT)
            assert charge.wait_for_service(timeout_sec=10), 'X30 charge service missing'

            def request(command):
                return charge.call_async(SetParameters.Request(parameters=[Parameter(
                    name='charge_command', value=ParameterValue(type=2, integer_value=command))]))

            def result(future, timeout=2):
                rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
                assert future.done(), 'Request did not finish within its bound'
                return future.result().results[0]

            # No UDP response: the charge worker waits, ROS services still run.
            pending = request(4)
            health = parameters.call_async(GetParameters.Request(names=['charge_command_timeout_ms']))
            rclpy.spin_until_future_complete(node, health, timeout_sec=.4)
            assert health.done(), 'Charge wait blocked ROS callbacks'
            failed = result(pending)
            assert not failed.successful and 'Timed out' in failed.reason

            # Reproduce the field status: unknown 0x100B is not success and cannot hang.
            state = 0x100B
            failed = result(request(4))
            assert not failed.successful and '4107' in failed.reason, failed.reason

            # A queued QUERY cannot overtake STOP and falsely prove its completion.
            before = len(packets)
            pending = request(4)
            time.sleep(.15)  # QUERY has its own later deadline while still joining the queue.
            barrier = request(3)
            failed = result(pending)
            assert not failed.successful
            assert result(barrier).successful
            charge_packets = [p for p in packets[before:] if p[0] in (0x91910250, 0x91910253)]
            assert charge_packets[0][:2] == (0x91910250, 0)
            assert charge_packets[-1][0] == 0x91910253
            assert charge_packets[-1][2] - charge_packets[0][2] >= .6

            # Firmware recovery permits the next command without restarting either node.
            state = 0
            assert result(request(4)).successful
            assert result(request(3)).successful

            # Bound admission as well as execution, including the time spent in queue.
            state = None
            futures = [request(4) for _ in range(12)]
            outcomes = [result(future) for future in futures]
            assert all(not item.successful for item in outcomes)
            assert any('queue is full' in item.reason for item in outcomes)
            state = 0
            assert result(request(4)).successful
            print('X30: no response, unknown status, ROS responsiveness, serial barrier, queue bound and retry passed')
        except BaseException:
            log.seek(0)
            print(log.read())
            raise
        finally:
            if process is not None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            node.destroy_node()
            rclpy.shutdown()
            stopping.set()
            thread.join(timeout=2)
            udp.close()


if __name__ == '__main__':
    main()
