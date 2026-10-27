#!/usr/bin/env python3
"""Exercise the ROS/HTTP contract against loopback, never real hardware."""
import argparse
import json
import math
import os
import signal
import socket
import statistics
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

os.environ['ROS_DOMAIN_ID'] = '229'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain Id="any"><General>'
    '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
    '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>')

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters


class Robot:
    def __init__(self):
        self.lock = threading.Lock()
        self.calls = []
        self.brake = True
        self.offline = False
        self.reject_move = False
        self.reject_release = False
        self.release_http_error = False
        self.move_delay_once = 0.0
        self.move_delay = 0.0
        self.state_delay_once = 0.0
        self.connections = set()
        self.battery_delay = 0.0
        self.invalid_battery = False
        self.invalid_raw_velocity = False
        self.raw_velocity_delay = 0.0
        self.state = 'Idle'
        self.ready_delay = 0.0

    def snapshot(self):
        with self.lock:
            return list(self.calls)

    def handle(self, path, body):
        with self.lock:
            self.calls.append((time.monotonic(), path, body))
            delay = 0.0
            status = 200
            result = {'successed': True, 'errorCode': '', 'msg': '', 'data': {}}
            if self.offline:
                status = 503
            elif path.startswith('/robot/cmd/change_robot_state'):
                assert path.endswith('?state=Idle'), path
                self.state = 'Idle'
                delay = self.ready_delay
            elif path == '/robot/cmd/get_robot_state':
                delay, self.state_delay_once = self.state_delay_once, 0.0
                # Real device returns false in Idle despite a successful API response.
                result['data'] = {'state_initialized': False, 'state_operation': self.state}
            elif path == '/robot/cmd/get_move_brake':
                result['data'] = {'brake_state': self.brake}
            elif path == '/robot/cmd/move_brake':
                if self.release_http_error and not body['brake']:
                    status = 500
                    result['msg'] = 'fixture brake release failure\nserver detail'
                elif self.reject_release and not body['brake']:
                    result['successed'] = False
                    result['errorCode'] = 100
                else:
                    self.brake = body['brake']
            elif path == '/robot/cmd/move':
                delay = self.move_delay_once or self.move_delay
                self.move_delay_once = 0.0
                if self.reject_move:
                    result['successed'] = False
                    result['errorCode'] = 100
                # Acknowledgements deliberately do not claim measured velocity.
            elif path == '/robot/sensor_data/battery':
                delay = self.battery_delay
                result['data'] = '' if self.invalid_battery else {'battery_capacity_percentage': 76.0}
            elif path == '/robot/sensor_data/raw_velocity':
                delay = self.raw_velocity_delay
                result['data'] = 'invalid' if self.invalid_raw_velocity else {
                    'device_type': 'motor_vel', 'device_id': '1',
                    'device_data': {'linear_vel': .17, 'linear_y': -.02, 'angular_rotate': -.31}}
            else:
                status = 404
        if delay:
            time.sleep(delay)
        return status, result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    robot = Robot()

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def setup(self):
            super().setup()
            self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def do_POST(self):
            with robot.lock:
                robot.connections.add(self.client_address)
            raw = self.rfile.read(int(self.headers.get('Content-Length', 0)))
            body = json.loads(raw) if raw else {}
            status, result = robot.handle(self.path, body)
            encoded = json.dumps(result, indent=2).encode()
            try:
                self.send_response(status)
                self.send_header('Content-Type', 'application/json')
                # Exercise libcurl's chunked response handling as well as JSON whitespace.
                self.send_header('Transfer-Encoding', 'chunked')
                self.end_headers()
                for index in range(0, len(encoded), 17):
                    chunk = encoded[index:index + 17]
                    self.wfile.write(f'{len(chunk):x}\r\n'.encode() + chunk + b'\r\n')
                self.wfile.write(b'0\r\n\r\n')
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    parameters = [
        'host_ip:=127.0.0.1', f'host_port:={server.server_port}',
        'cmd_vel_rate_hz:=10.0', 'cmd_vel_timeout_ms:=250',
        'http_timeout_ms:=500', 'connect_timeout_ms:=100', 'move_timeout_ms:=120',
        'state_poll_interval_ms:=100', 'battery_poll_interval_ms:=200',
        'reconnect_interval_ms:=250', 'telemetry_timeout_ms:=2000',
    ]
    command = [args.executable, '--ros-args']
    for parameter in parameters:
        command += ['-p', parameter]
    output = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('tank_protocol_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    command_value = [None]
    diagnostics = [{}]
    batteries = []
    raw_velocities = []

    def publish():
        if command_value[0] is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = command_value[0]
            pub.publish(msg)

    timer = node.create_timer(.025, publish)
    subscriptions = [
        node.create_subscription(String, '/nav_bridge_node/backend_status',
                                 lambda msg: diagnostics.__setitem__(0, json.loads(msg.data)), 10),
        node.create_subscription(UInt8, '/battery/level', lambda msg: batteries.append(msg.data), 10),
        node.create_subscription(Twist, '/raw_vel', lambda msg: raw_velocities.append(
            (msg.linear.x, msg.linear.y, msg.angular.z)), 10),
    ]
    clients = {name: node.create_client(Trigger, '/nav_bridge_node/' + name)
               for name in ('ready', 'stand', 'lie', 'soft_estop', 'release_control')}

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None, 'TANK node exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=.02)

    def wait_for(predicate, description, timeout=5.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            spin_for(.025)
        raise AssertionError(description + ': ' + str(diagnostics[0]))

    def call(name, success=True):
        client = clients[name]
        wait_for(client.service_is_ready, f'{name} service discovery')
        future = client.call_async(Trigger.Request())
        wait_for(future.done, f'{name} response', 8.)
        result = future.result()
        assert result.success == success, (name, result)
        return result

    def moves(since=0.):
        return [(stamp, data['speed']) for stamp, path, data in robot.snapshot()
                if path == '/robot/cmd/move' and stamp >= since]

    def nonzero(since=0.):
        return [(stamp, data) for stamp, data in moves(since)
                if abs(data['linearSpeed']) > 1e-6 or abs(data['angularSpeed']) > 1e-6]

    try:
        wait_for(lambda: diagnostics[0].get('connected'), 'initial connection')
        assert diagnostics[0]['control_owner'] == 'unknown'
        assert diagnostics[0]['control_owner_known'] is False
        assert diagnostics[0]['robot_controlled_by_bridge'] is None
        assert diagnostics[0]['bridge_output_enabled'] == diagnostics[0]['control_owned']
        assert not diagnostics[0]['control_owned'], 'Startup must not auto-release existing brake'
        assert robot.brake
        wait_for(lambda: raw_velocities, 'raw feedback while startup brake enabled')
        assert raw_velocities[-1] == (.17, -.02, -.31), raw_velocities
        call('ready')
        wait_for(lambda: diagnostics[0].get('control_owned'), 'ready control enabled')
        assert not robot.brake
        wait_for(lambda: 76 in batteries, 'battery telemetry')
        robot.invalid_raw_velocity = True
        wait_for(lambda: diagnostics[0].get('last_raw_vel_error'), 'invalid raw feedback diagnostic')
        spin_for(.15)  # Drain messages from before the invalid response.
        count_before = len(raw_velocities)
        spin_for(.3)
        assert len(raw_velocities) == count_before, 'invalid feedback fabricated or repeated velocity'
        assert diagnostics[0]['connected'] and diagnostics[0]['control_owned']
        robot.invalid_raw_velocity = False
        wait_for(lambda: len(raw_velocities) > count_before and diagnostics[0].get('raw_vel_valid'),
                 'raw velocity recovery')
        robot.raw_velocity_delay = .65
        wait_for(lambda: 'timed out' in diagnostics[0].get('last_raw_vel_error', ''), 'raw velocity timeout')
        assert diagnostics[0]['connected'] and diagnostics[0]['control_owned']
        robot.raw_velocity_delay = 0.
        wait_for(lambda: not diagnostics[0].get('last_raw_vel_error'), 'raw timeout recovery')

        # A released brake needs no write, even if its release handler is broken.
        robot.release_http_error = True
        before = len([p for _, p, _ in robot.snapshot() if p == '/robot/cmd/move_brake'])
        call('ready')
        call('stand')
        after = len([p for _, p, _ in robot.snapshot() if p == '/robot/cmd/move_brake'])
        assert before == after, 'already released brake received a redundant release request'
        robot.release_http_error = False

        start = time.monotonic()
        command_value[0] = (3.0, .8, .4)
        spin_for(.8)
        samples = nonzero(start)
        assert len(samples) >= 5, samples
        assert all(abs(v['linearSpeed'] - 1.2) < 1e-6 and
                   abs(v['angularSpeed'] - .4) < 1e-6 for _, v in samples), samples
        intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        assert .07 < statistics.median(intervals) < .14, intervals
        command_value[0] = None
        spin_for(.6)
        assert moves()[-1][1] == {'linearSpeed': 0., 'angularSpeed': 0.}, 'stale velocity must zero'

        command_value[0] = (.2, 0., -.3)
        spin_for(.3)
        assert abs(nonzero()[-1][1]['angularSpeed'] + .3) < 1e-6, 'negative ROS yaw must remain negative'
        command_value[0] = (0., 0., -3.)
        spin_for(.3)
        assert nonzero()[-1][1] == {'linearSpeed': 0., 'angularSpeed': -1.}, 'pure rotation must preserve sign and clamp magnitude'
        command_value[0] = (math.nan, 0., 0.)
        spin_for(.3)
        assert moves()[-1][1] == {'linearSpeed': 0., 'angularSpeed': 0.}, 'non-finite input must clear previous velocity'
        command_value[0] = (.2, 0., .1)
        spin_for(.2)
        call('lie')
        stop_time = time.monotonic()
        spin_for(.4)
        assert not nonzero(stop_time), 'lie must inhibit fresh commands until ready'
        command_value[0] = None
        call('stand')
        spin_for(.4)
        assert not nonzero(stop_time), 'stand must not replay old command'

        command_value[0] = (.2, 0., 0.)
        spin_for(.25)
        call('soft_estop')
        assert robot.brake
        stop_time = time.monotonic()
        spin_for(.3)
        assert not nonzero(stop_time), 'estop must inhibit motion'
        command_value[0] = None
        robot.release_http_error = True
        error = call('ready', success=False)
        assert 'HTTP 500' in error.message and 'fixture brake release failure' in error.message, error
        assert '\n' not in error.message, 'server response must be escaped into one log line'
        assert robot.brake
        robot.release_http_error = False
        robot.reject_release = True
        call('ready', success=False)
        assert robot.brake, 'failed ready must not pretend brake release'
        robot.reject_release = False
        call('ready')
        assert not robot.brake

        # A slow battery query must not stall control streaming or ROS estop.
        robot.battery_delay = .3
        command_value[0] = (.25, 0., 0.)
        start = time.monotonic()
        spin_for(.7)
        assert len(nonzero(start)) >= 5, 'slow telemetry blocked control'
        before = time.monotonic()
        call('soft_estop')
        assert time.monotonic() - before < .7, 'estop blocked by telemetry'
        robot.battery_delay = 0.
        command_value[0] = None
        call('ready')

        # An external brake revokes local control; polling cannot release it.
        robot.brake = True
        wait_for(lambda: not diagnostics[0].get('control_owned', True), 'external brake inhibits motion')
        spin_for(.3)
        assert robot.brake
        call('ready')

        command_value[0] = (.25, 0., 0.)
        spin_for(.2)
        call('release_control')
        stop_time = time.monotonic()
        spin_for(.4)
        assert not nonzero(stop_time), 'release must inhibit motion'
        command_value[0] = None
        call('ready')

        # Stop invalidates an in-progress ready before it can re-enable motion.
        robot.ready_delay = .3
        pending = clients['ready'].call_async(Trigger.Request())
        spin_for(.1)
        call('soft_estop')
        wait_for(pending.done, 'cancelled ready response')
        assert not pending.result().success
        assert robot.brake
        robot.ready_delay = 0.
        call('ready')

        # An isolated transport timeout clears cached velocity but does not
        # switch Idle/reconnect. Fresh input is required to move again.
        command_value[0] = (.3, 0., 0.)
        spin_for(.2)
        command_value[0] = None
        mode_calls = lambda: sum('/change_robot_state' in path for _, path, _ in robot.snapshot())
        modes_before = mode_calls()
        robot.move_delay_once = .35
        wait_for(lambda: diagnostics[0].get('move_failures', 0) > 0, 'move timeout counted')
        wait_for(lambda: diagnostics[0].get('consecutive_move_failures') == 0, 'motion transport recovery')
        assert diagnostics[0]['connected'] and diagnostics[0]['control_owned']
        assert mode_calls() == modes_before, 'isolated timeout triggered mode change'
        recovery_time = time.monotonic()
        spin_for(.4)
        assert not nonzero(recovery_time), 'transport recovery replayed stale velocity'

        robot.state_delay_once = .65
        wait_for(lambda: diagnostics[0].get('consecutive_state_failures', 0) > 0, 'state timeout counted')
        wait_for(lambda: diagnostics[0].get('consecutive_state_failures') == 0, 'state transport recovery')
        assert diagnostics[0]['connected'] and mode_calls() == modes_before

        # Persistent transport failures still revoke control.
        robot.move_delay = .2
        wait_for(lambda: not diagnostics[0].get('connected', True), 'persistent move timeout disconnects')
        robot.move_delay = 0.
        wait_for(lambda: diagnostics[0].get('connected'), 'persistent timeout recovery')

        # HTTP 200 with successed=false is a business failure, not success.
        robot.reject_move = True
        wait_for(lambda: not diagnostics[0].get('connected', True), 'business rejection disconnects')
        robot.reject_move = False
        wait_for(lambda: diagnostics[0].get('connected'), 'reconnect after rejection')
        robot.invalid_battery = True
        spin_for(.5)
        assert process.poll() is None, 'invalid battery data crashed node'
        robot.invalid_battery = False
        robot.offline = True
        wait_for(lambda: not diagnostics[0].get('connected', True), 'HTTP offline detection')
        robot.offline = False
        wait_for(lambda: diagnostics[0].get('connected'), 'HTTP offline recovery')

        unsupported = node.create_client(SetParameters, '/nav_bridge_node/set_gait')
        wait_for(unsupported.service_is_ready, 'unsupported service discovery')
        req = SetParameters.Request()
        req.parameters = [Parameter(name='gait', value=ParameterValue(type=2, integer_value=0))]
        future = unsupported.call_async(req)
        wait_for(future.done, 'unsupported response')
        assert not future.result().results[0].successful
        node.destroy_client(unsupported)

        # Separate instance: parameterized 20 Hz and disabled startup ownership.
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        rate_command = command + ['-p', 'cmd_vel_rate_hz:=20.0', '-p', 'startup_acquire_control:=false',
                                  '-p', 'move_timeout_ms:=1000']
        process = subprocess.Popen(rate_command, stdout=output, stderr=subprocess.STDOUT)
        robot.brake = False
        diagnostics[0] = {}
        wait_for(lambda: diagnostics[0].get('connected'), 'second instance connected')
        assert not diagnostics[0]['control_owned']
        call('ready')
        start = time.monotonic()
        command_value[0] = (.2, 0., 0.)
        spin_for(.6)
        samples = nonzero(start)
        assert len(samples) >= 8, 'configurable 20 Hz control rate not applied'
        intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        assert .035 < statistics.median(intervals) < .075, intervals
        connections_before = len(robot.connections)
        robot.move_delay = .08
        start = time.monotonic()
        spin_for(1.0)
        samples = nonzero(start)
        intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        assert len(intervals) >= 7 and .07 < statistics.median(intervals) < .115, intervals
        assert 8. < diagnostics[0]['actual_move_rate_hz'] < 21., diagnostics[0]
        assert len(robot.connections) <= connections_before + 1, 'HTTP connections not reused'
        robot.move_delay = 0.
        # Slower than the previous 200 ms limit, but within the new request budget.
        robot.move_delay_once = .35
        spin_for(.7)
        assert diagnostics[0]['connected'] and diagnostics[0]['move_failures'] == 0, diagnostics[0]
        command_value[0] = None
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        assert moves()[-1][1] == {'linearSpeed': 0., 'angularSpeed': 0.}, 'shutdown must send zero'

        # Query-only mode must never touch any control endpoint, even during
        # reconnect, service calls, or shutdown. It can observe non-Idle states.
        robot.state = 'Mapping'
        diagnostics[0] = {}
        readonly_start = time.monotonic()
        process = subprocess.Popen(command + ['-p', 'enable_motion_output:=false'],
                                   stdout=output, stderr=subprocess.STDOUT)
        wait_for(lambda: diagnostics[0].get('connected') and
                 diagnostics[0].get('enable_motion_output') is False, 'query-only connection')
        assert diagnostics[0]['device_state'] == 'Mapping'
        assert not diagnostics[0]['state_initialized']
        assert not diagnostics[0]['enable_motion_output']
        assert not diagnostics[0]['control_owned']
        for name in clients:
            call(name, success=False)
        command_value[0] = (.4, 0., .2)
        spin_for(.3)
        command_value[0] = None
        robot.offline = True
        wait_for(lambda: not diagnostics[0].get('connected', True), 'query-only offline')
        robot.offline = False
        wait_for(lambda: diagnostics[0].get('connected'), 'query-only reconnect')
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        readonly_calls = [path for stamp, path, _ in robot.snapshot() if stamp >= readonly_start]
        assert readonly_calls
        assert set(readonly_calls) <= {'/robot/cmd/get_robot_state', '/robot/cmd/get_move_brake',
                                     '/robot/sensor_data/battery', '/robot/sensor_data/raw_velocity'}, readonly_calls

        invalid = subprocess.run(command + ['-p', 'cmd_vel_rate_hz:=2.0'],
                                 stdout=output, stderr=subprocess.STDOUT, timeout=5)
        assert invalid.returncode != 0, 'invalid protocol frequency accepted'
        print('TANK protocol integration passed: control, estop, concurrency, telemetry, reconnect, parameters')
    except BaseException:
        output.seek(0)
        print(output.read())
        raise
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        node.destroy_timer(timer)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        output.close()


if __name__ == '__main__':
    main()
