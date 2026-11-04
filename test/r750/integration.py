#!/usr/bin/env python3
"""Exercise the ROS/serial contract against a pty mock, never real hardware."""
import argparse
import json
import math
import os
import select
import signal
import statistics
import subprocess
import tempfile
import termios
import threading
import time

os.environ['ROS_DOMAIN_ID'] = '231'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = (
    '<CycloneDDS><Domain Id="any"><General>'
    '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
    '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>')

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Int32, String, UInt8
from std_srvs.srv import Trigger

SYNC = b'\xAA\x55'
ZERO_FRAME = bytes([0xAA, 0x55, 0x0B, 0x50, 0, 0, 0, 0, 0, 0, 0x5A])


def frame(frame_id, payload):
    body = SYNC + bytes([len(payload) + 5, frame_id]) + payload
    return body + bytes([sum(body) & 0xFF])


def velocity_frame(vx, vy, vyaw):
    return frame(0x50, put_int16(vx * 1000) + put_int16(vy * 1000) + put_int16(vyaw * 1000))


def telemetry_frame(vx, vy, vyaw, centivolts):
    payload = bytearray(12)  # IMU block: the backend parses past it, keep zero
    payload += put_int16(vx * 1000) + put_int16(vy * 1000) + put_int16(vyaw * 1000)
    payload += int(centivolts).to_bytes(2, 'big')
    return frame(0x10, bytes(payload))


def put_int16(value):
    return max(-32768, min(32767, int(round(value)))).to_bytes(2, 'big', signed=True)


def decode(raw):
    return (int.from_bytes(raw[4:6], 'big', signed=True) / 1000.0,
            int.from_bytes(raw[6:8], 'big', signed=True) / 1000.0,
            int.from_bytes(raw[8:10], 'big', signed=True) / 1000.0)


def battery_percent(voltage):
    # Mirrors the backend: (V - empty) / (full - empty) * 100, clamped, truncated.
    fraction = (voltage - 9.9) / (12.6 - 9.9) * 100.0
    return int(min(100.0, max(0.0, fraction)))


class MockMcu:
    """50 Hz X-Protocol MCU over a pty; records every byte the bridge writes."""

    def __init__(self, vx=0.17, vy=-.02, vyaw=-.31, voltage=12.6):
        self.master, slave = os.openpty()
        self.slave_path = os.ttyname(slave)
        # Raw line discipline on the shared slave settings: no echo of the
        # telemetry stream, no ONLCR mangling of the bridge's output bytes.
        attrs = termios.tcgetattr(slave)
        attrs[0] = 0
        attrs[1] = 0
        attrs[3] = 0
        termios.tcsetattr(slave, termios.TCSANOW, attrs)
        self.slave = slave  # kept open so the settings survive reconnects
        self.lock = threading.Lock()
        self.vx, self.vy, self.vyaw = vx, vy, vyaw
        self.centivolts = int(round(voltage * 100))
        self.frames = []  # (time, raw bytes) of every decoded 0x50 frame
        self.rx_bytes = 0
        self.checksum_errors = 0
        self.running = True
        self._count = 0
        self._checksum = 0
        self._length = 0
        self._buffer = bytearray()
        self.workers = [threading.Thread(target=self._telemetry_loop, daemon=True),
                        threading.Thread(target=self._receive_loop, daemon=True)]
        for worker in self.workers:
            worker.start()

    def set_measured(self, vx, vy, vyaw):
        with self.lock:
            self.vx, self.vy, self.vyaw = vx, vy, vyaw

    def set_voltage(self, volts):
        with self.lock:
            self.centivolts = int(round(volts * 100))

    def _telemetry_loop(self):
        while self.running:
            with self.lock:
                sample = (self.vx, self.vy, self.vyaw, self.centivolts)
            try:
                os.write(self.master, telemetry_frame(*sample))
            except OSError:
                return
            time.sleep(.02)

    def _receive_loop(self):
        while self.running:
            try:
                ready, _, _ = select.select([self.master], [], [], .05)
            except (OSError, ValueError):
                return
            if not ready:
                continue
            try:
                data = os.read(self.master, 4096)
            except OSError:
                return
            if not data:
                return
            self.rx_bytes += len(data)
            self._parse(data)

    def _parse(self, data):
        for byte in data:
            if self._count < 3:
                if self._count == 0:
                    if byte == 0xAA:
                        self._count = 1
                elif self._count == 1:
                    self._count = 2 if byte == 0x55 else 0
                else:
                    if byte < 5 or byte > 64:
                        self._count = 0
                        self.checksum_errors += 1
                    else:
                        self._length = byte
                        self._count = 3
                        self._checksum = (0xAA + 0x55 + byte) & 0xFF
            elif self._count < self._length - 1:
                self._buffer.append(byte)
                self._count += 1
                self._checksum = (self._checksum + byte) & 0xFF
            else:
                complete = bytes([0xAA, 0x55, self._length]) + bytes(self._buffer) + bytes([byte])
                self._count = 0
                self._buffer = bytearray()
                if byte == self._checksum:
                    if complete[3] == 0x50:
                        self.frames.append((time.monotonic(), complete))
                else:
                    self.checksum_errors += 1

    def stop(self):
        if not self.running:
            return
        self.running = False
        for worker in self.workers:
            worker.join(timeout=2)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def frames_since(mock, since=0.):
    return [(stamp, raw) for stamp, raw in mock.frames if stamp >= since]


def nonzero_since(mock, since=0.):
    return [(stamp, raw) for stamp, raw in frames_since(mock, since)
            if any(abs(value) > 1e-6 for value in decode(raw))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()

    mock = MockMcu()
    link = f'/tmp/r750_serial_{os.getpid()}'
    os.symlink(mock.slave_path, link)
    base_parameters = ['serial_baud:=230400', 'connect_timeout_ms:=3000',
                       'reconnect_interval_ms:=400', 'telemetry_timeout_ms:=800',
                       'cmd_vel_rate_hz:=10.0', 'cmd_vel_timeout_ms:=250',
                       'max_linear_speed:=0.7', 'max_angular_speed:=0.8',
                       'state_publish_rate_hz:=10.0']

    def node_command(port, extra=()):
        argv = [args.executable, '--ros-args', '-p', f'serial_port:={port}']
        for parameter in list(base_parameters) + list(extra):
            argv += ['-p', parameter]
        return argv

    output = tempfile.TemporaryFile(mode='w+')
    mock2 = mock3 = mock4 = None
    link3 = link4 = None
    process = subprocess.Popen(node_command(link), stdout=output, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('r750_protocol_test')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    command_value = [None]
    diagnostics = [{}]
    batteries = []
    raw_velocities = []
    basic_states = [-1]

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
        node.create_subscription(Int32, '/robot_basic_state',
                                 lambda msg: basic_states.__setitem__(0, msg.data), 10),
    ]
    clients = {name: node.create_client(Trigger, '/nav_bridge_node/' + name)
               for name in ('ready', 'stand', 'lie', 'soft_estop', 'release_control')}

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None, 'R750 node exited unexpectedly'
            rclpy.spin_once(node, timeout_sec=.02)

    def wait_for(predicate, description, timeout=8.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            spin_for(.025)
        raise AssertionError(description + ': ' + json.dumps(diagnostics[0]))

    def call(name, success=True):
        client = clients[name]
        wait_for(client.service_is_ready, f'{name} service discovery')
        future = client.call_async(Trigger.Request())
        wait_for(future.done, f'{name} response')
        result = future.result()
        assert result.success == success, (name, result)
        return result

    def check_inhibition(stop_name, stopped_state):
        command_value[0] = (.2, 0., 0.)
        wait_for(lambda: nonzero_since(mock), f'moving before {stop_name}')
        command_value[0] = None
        wait_for(lambda: decode(mock.frames[-1][1]) == (0., 0., 0.), f'zeros before {stop_name}')
        stop_time = time.monotonic()
        call(stop_name)
        if stop_name == 'soft_estop':
            wait_for(lambda: diagnostics[0].get('local_estop') is True, 'estop latch reported')
        after_stop = frames_since(mock, stop_time)
        assert after_stop and all(decode(raw) == (0., 0., 0.) for _, raw in after_stop), after_stop
        quiet = stop_time + .25
        assert not frames_since(mock, quiet), f'{stop_name} must stop streaming entirely'
        command_value[0] = (.3, 0., 0.)
        spin_for(.35)
        assert not frames_since(mock, quiet), f'{stop_name} must ignore fresh cmd_vel'
        command_value[0] = None
        stand_time = time.monotonic()
        call('stand')
        wait_for(lambda: frames_since(mock, stand_time), 'stand sends zero velocity')
        spin_for(.4)
        resumed = frames_since(mock, stand_time)
        assert len(resumed) >= 3 and all(decode(raw) == (0., 0., 0.) for _, raw in resumed)
        command_value[0] = (.2, 0., 0.)
        wait_for(lambda: nonzero_since(mock, stand_time), 'motion resumes after stand')

    try:
        wait_for(lambda: diagnostics[0].get('connected'), 'initial connection')
        assert diagnostics[0]['robot_type'] == 'r750'
        assert diagnostics[0]['control_owner'] == 'unknown'
        assert diagnostics[0]['control_owner_known'] is False
        assert diagnostics[0]['robot_controlled_by_bridge'] is None
        assert diagnostics[0]['bridge_output_enabled'] == diagnostics[0]['control_owned']
        assert diagnostics[0]['control_owned'], 'startup_acquire_control must arm output'

        # (1) startup zero-velocity streaming and exact frame encoding.
        wait_for(lambda: mock.frames, 'startup zero-velocity streaming')
        assert mock.frames[0][1] == ZERO_FRAME, mock.frames[0][1]
        assert all(raw == ZERO_FRAME for _, raw in mock.frames), 'startup must stream zeros only'

        # (7) measured velocity telemetry from the 0x10 frames.
        wait_for(lambda: raw_velocities, 'measured velocity telemetry')
        assert abs(raw_velocities[-1][0] - .17) < 1e-6
        assert abs(raw_velocities[-1][1] + .02) < 1e-6
        assert abs(raw_velocities[-1][2] + .31) < 1e-6
        wait_for(lambda: basic_states[0] == 3, 'standing classification while streaming zeros')

        # (6) battery curve from mock voltage, including both clamps.
        for voltage in (12.6, 11.25, 10.5, 8.4, 13.2):
            mock.set_voltage(voltage)
            expected = battery_percent(voltage)
            wait_for(lambda e=expected: e in batteries, f'battery {voltage}V -> {expected}%',
                     timeout=5.)

        # (1) nonzero frame encoding: scaling, big-endian, length, checksum.
        command_value[0] = (.2, .123, -.35)
        wait_for(lambda: nonzero_since(mock), 'nonzero command streaming')
        raw = mock.frames[-1][1]
        assert len(raw) == 11 and raw[2] == 11 and raw[3] == 0x50, raw
        assert raw[4:6] == (200).to_bytes(2, 'big'), raw
        assert raw[6:8] == b'\x00\x00', 'lateral input must be ignored'
        assert raw[8:10] == (-350).to_bytes(2, 'big', signed=True), raw
        assert raw[10] == sum(raw[:10]) & 0xFF, raw
        wait_for(lambda: basic_states[0] == 4, 'moving classification')
        start = time.monotonic()
        spin_for(.8)
        samples = nonzero_since(mock, start)
        assert len(samples) >= 5, samples
        intervals = [b[0] - a[0] for a, b in zip(samples, samples[1:])]
        assert .07 < statistics.median(intervals) < .14, intervals

        # (2) clamping at the configured speed limits.
        command_value[0] = (5.0, 9.0, 4.0)
        wait_for(lambda: decode(mock.frames[-1][1])[0] > .69, 'clamped linear command')
        assert decode(mock.frames[-1][1]) == (.7, 0., .8), mock.frames[-1]
        command_value[0] = (0., 0., -3.)
        wait_for(lambda: decode(mock.frames[-1][1]) == (0., 0., -.8), 'clamped angular sign kept')

        # Non-finite input clears the previous command.
        command_value[0] = (math.nan, 0., 0.)
        spin_for(.35)
        assert decode(mock.frames[-1][1]) == (0., 0., 0.), 'non-finite must clear the command'
        command_value[0] = None

        # (3) cmd_vel timeout keeps streaming zero velocity.
        command_value[0] = (.25, 0., 0.)
        wait_for(lambda: nonzero_since(mock), 'nonzero before timeout test')
        mark = len(mock.frames)
        command_value[0] = None
        wait_for(lambda: len(mock.frames) >= mark + 3 and
                 all(decode(raw) == (0., 0., 0.) for _, raw in mock.frames[mark + 1:]),
                 'stale input must stream zero velocity')
        wait_for(lambda: basic_states[0] == 3, 'standing after timeout zeroing')

        # (4) lie / soft_estop / release_control block until stand.
        check_inhibition('lie', 5)
        check_inhibition('soft_estop', 6)
        check_inhibition('release_control', 5)

        unsupported = node.create_client(SetParameters, '/nav_bridge_node/set_gait')
        wait_for(unsupported.service_is_ready, 'unsupported service discovery')
        req = SetParameters.Request()
        req.parameters = [Parameter(name='gait', value=ParameterValue(type=2, integer_value=0))]
        future = unsupported.call_async(req)
        wait_for(future.done, 'unsupported response')
        answer = future.result().results[0]
        assert not answer.successful
        assert answer.reason == 'R750 set_gait is unsupported in this backend', answer.reason
        node.destroy_client(unsupported)

        # (8) reconnect through a retargeted symlink; no motion until stand.
        command_value[0] = None
        spin_for(.25)
        mock.stop()
        mock2 = MockMcu(voltage=11.25)
        retarget = link + '.next'
        os.symlink(mock2.slave_path, retarget)
        os.replace(retarget, link)
        wait_for(lambda: not diagnostics[0].get('connected', True), 'link loss detected')
        assert not diagnostics[0].get('control_owned', False)
        wait_for(lambda: diagnostics[0].get('connected'), 'reconnect through retargeted port')
        spin_for(.7)
        assert not mock2.frames, 'reconnect must not resume motion output'
        command_value[0] = (.3, 0., 0.)
        spin_for(.45)
        assert not mock2.frames, 'fresh cmd_vel must stay inhibited until ready/stand'
        command_value[0] = None
        call('stand')
        stand_time = time.monotonic()
        wait_for(lambda: frames_since(mock2, stand_time), 'stand re-arms output after reconnect')
        command_value[0] = (.25, 0., 0.)
        wait_for(lambda: nonzero_since(mock2, stand_time), 'motion after reconnect and stand')

        # Teardown mirrors the tarkbot driver: one final zero-velocity frame.
        command_value[0] = None
        marker = len(mock2.frames)
        command_value[0] = (.2, 0., 0.)
        wait_for(lambda: any(stamp >= 0 and index >= marker and any(abs(v) > 1e-6 for v in decode(raw))
                             for index, (stamp, raw) in enumerate(mock2.frames)),
                 'moving before shutdown')
        shutdown_time = time.monotonic()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        assert mock2.frames and mock2.frames[-1][1] == ZERO_FRAME, 'shutdown must send zero'
        assert mock2.frames[-1][0] >= shutdown_time - .05
        command_value[0] = None

        # (9) startup_acquire_control:=false sends nothing until ready.
        mock3 = MockMcu()
        link3 = link + '3'
        os.symlink(mock3.slave_path, link3)
        diagnostics[0] = {}
        process = subprocess.Popen(node_command(link3, ['startup_acquire_control:=false']),
                                   stdout=output, stderr=subprocess.STDOUT)
        wait_for(lambda: diagnostics[0].get('connected'), 'deferred-control connection')
        assert not diagnostics[0]['control_owned']
        spin_for(.6)
        assert mock3.rx_bytes == 0, 'deferred startup must not stream'
        call('ready')
        wait_for(lambda: mock3.frames, 'ready starts zero streaming')
        assert mock3.frames[0][1] == ZERO_FRAME
        command_value[0] = (.2, 0., 0.)
        wait_for(lambda: nonzero_since(mock3), 'motion after explicit ready')
        shutdown_time = time.monotonic()
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        assert mock3.frames[-1][1] == ZERO_FRAME, 'shutdown must send zero'
        command_value[0] = None

        # (5) enable_motion_output:=false never writes a byte, even on teardown.
        mock4 = MockMcu()
        link4 = link + '4'
        os.symlink(mock4.slave_path, link4)
        diagnostics[0] = {}
        raw_before = len(raw_velocities)
        process = subprocess.Popen(node_command(link4, ['enable_motion_output:=false']),
                                   stdout=output, stderr=subprocess.STDOUT)
        wait_for(lambda: diagnostics[0].get('connected') and
                 diagnostics[0].get('enable_motion_output') is False, 'query-only connection')
        assert not diagnostics[0]['control_owned']
        wait_for(lambda: len(raw_velocities) > raw_before, 'query-only telemetry still observed')
        for name in clients:
            call(name, success=False)
        command_value[0] = (.4, 0., .2)
        spin_for(.4)
        command_value[0] = None
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5)
        assert process.returncode == 0
        assert mock4.rx_bytes == 0, 'query-only mode must never write, not even on teardown'

        invalid = subprocess.run(node_command('/dev/null', ['cmd_vel_rate_hz:=2.0']),
                                 stdout=output, stderr=subprocess.STDOUT, timeout=8)
        assert invalid.returncode != 0, 'invalid protocol frequency accepted'
        print('R750 protocol integration passed: framing, clamps, timeout, estop gating, '
              'readonly, battery curve, measured velocity, reconnect, parameters')
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
        for stopped in (mock, mock2, mock3, mock4):
            if stopped:
                stopped.stop()
        for path in (link, link + '.next', link3, link4):
            if path and os.path.islink(path):
                os.unlink(path)
        output.close()



if __name__ == '__main__':
    main()
