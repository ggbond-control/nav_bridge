#!/usr/bin/env python3
"""ROS protocol fixture for Cyvet. Uses isolated domains and never real hardware."""
import argparse
import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time

# Tests are confined to loopback and unrelated domains even if the caller
# sources the deployed hardware configuration.
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain Id="any"><General>'
    '<Interfaces><NetworkInterface name="lo"/></Interfaces>'
    '<AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>')

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from uniubi.srv import System
from uniubi.msg import MotionObserved, SensorObserved, EventMessage


class Robot:
    def __init__(self, node):
        self.node = node
        self.owner = ''
        self.lease_activity = time.monotonic()
        self.lease_ms = 5000
        self.action = ''
        self.velocity = [0., 0., 0.]
        self.offline = False
        self.reject_acquire = False
        self.reject_action = False
        self.reject_stop_count = 0
        self.calls = []
        self.enabled = False
        self.epoch = 1
        self.invalid_imu = False
        self.invalid_odom = False
        self.motor_count = 12
        self.wrong_identity = False
        self.delay_method = ''
        self.delay_seconds = 0.
        self.delay_remaining = -1
        self.reject_velocity = False
        self.wrong_velocity_identity = False
        self.invalid_quaternion = False
        self.motion_offline = False
        self.profile = 'slow'
        self.profile_lag_ticks = 0
        self.old_profile_ticks = 0
        self.previous_profile = 'slow'
        self.report_profile = False
        self.profile_override = ''
        self.reject_profile = ''
        self.action_params = []
        self.service = node.create_service(System, 'robotServer', self.request,
                                          callback_group=ReentrantCallbackGroup())
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.motion = node.create_publisher(MotionObserved, '/motion/observed', qos)
        self.sensor = node.create_publisher(SensorObserved, '/sensor/observed', qos)
        self.event = node.create_publisher(EventMessage, '/robotServer/Event', qos)
        self.timer = node.create_timer(.02, self.observed)

    def request(self, req, res):
        self.calls.append((req.method, req.device_id))
        res.device_id = ('different-robot' if self.wrong_identity and
                         req.method == 'setMotionMasterRole' else req.device_id)
        if req.method == self.delay_method and self.delay_remaining != 0:
            if self.delay_remaining > 0:
                self.delay_remaining -= 1
            time.sleep(self.delay_seconds)
        if self.offline or (self.motion_offline and req.method == 'queryMotionState') or req.device_id != 'cyvet-test':
            res.code = 1
            res.payload = '{}'
            return res
        value = json.loads(req.payload)
        p = value.get('params') or {}
        method = req.method
        result, params = True, {}
        if method == 'getMotionCapabilities':
            params = [{'name': a, 'params': [
                {'name': n, 'min': -cap, 'max': cap}
                for n, cap in zip(('lineVelocityX', 'lineVelocityY', 'velocity'), (.15, .08, .25))
            ] if a == 'walking' else []} for a in ('walking', 'laying', 'standing')]
        elif method == 'getMotorLayout':
            params = {'motors': [{'limbNo': i//3, 'jointNo': i % 3, 'name': f'joint_{i}'}
                                 for i in range(12)]}
        elif method == 'getSystemStatus':
            params = {'battery': {'online': True, 'power': 73.}}
        elif method == 'queryMotionState':
            params = {'action': self.action, **dict(zip(
                ('lineVelocityX', 'lineVelocityY', 'velocity'), self.velocity))}
            if self.report_profile:
                effective = self.previous_profile if self.old_profile_ticks else self.profile
                if self.old_profile_ticks:
                    self.old_profile_ticks -= 1
                params['controlProfile'] = self.profile_override or effective
        elif method == 'takeMotionControl':
            result = not self.reject_acquire
            if result:
                self.owner = 'fixture-controller'
                params = {'controller': self.owner, 'leaseTimeout': self.lease_ms}
        elif method == 'setMotionMasterRole':
            pass
        elif method == 'setMotionObservedEnable':
            self.enabled = p.get('motionEnable', False)
        elif method == 'releaseMotionControl':
            self.owner = ''
            event = EventMessage()
            event.magic = 0x53425645
            event.topic = 'robotServer.control.status'
            event.payload = json.dumps({'controlled': False})
            self.event.publish(event)
            time.sleep(.05)  # Real firmware can send this before the RPC ACK.
        elif method == 'renewMotionControl':
            result = bool(self.owner)
            params = {'leaseTimeout': self.lease_ms}
        elif method == 'startMotionAction':
            result = bool(self.owner) and not self.reject_action
            if result:
                self.action = p['action']
                self.velocity = [0., 0., 0.]
                if self.action == 'walking':
                    self.profile = p.get('params', {}).get('controlProfile', 'slow')
        elif method == 'setMotionActionParams':
            if self.wrong_velocity_identity:
                res.device_id = 'different-robot'
            action_params = p.get('params', {})
            self.action_params.append(dict(action_params))
            result = (bool(self.owner) and not self.reject_velocity and
                      action_params.get('controlProfile') != self.reject_profile)
            if result:
                if action_params.get('controlProfile', self.profile) != self.profile:
                    self.previous_profile = self.profile
                    self.old_profile_ticks = self.profile_lag_ticks
                self.profile = action_params.get('controlProfile', self.profile)
                self.velocity = [action_params.get(n, 0.)
                                 for n in ('lineVelocityX', 'lineVelocityY', 'velocity')]
        elif method in ('stopMotionAction', 'emergencyStopMotion'):
            result = bool(self.owner)
            if method == 'stopMotionAction' and self.reject_stop_count:
                self.reject_stop_count -= 1
                result = False
            if result:
                self.action = 'emergencyStop' if method == 'emergencyStopMotion' else 'walking'
                self.velocity = [0., 0., 0.]
        else:
            result = False
        if result and self.owner and method in (
                'takeMotionControl', 'renewMotionControl', 'startMotionAction',
                'setMotionActionParams', 'stopMotionAction', 'emergencyStopMotion'):
            self.lease_activity = time.monotonic()
        res.payload = json.dumps({'result': result, 'params': params})
        return res

    def observed(self):
        if self.owner and time.monotonic() - self.lease_activity > self.lease_ms/1000.:
            self.owner = ''
            self.velocity = [0., 0., 0.]
        if not self.enabled or self.offline:
            return
        m = MotionObserved()
        m.timestamp = time.monotonic_ns()
        m.power.power = 73.
        m.power.charge_voltage = 46.
        m.imu.quaternion.w = 0. if self.invalid_quaternion else 2.
        m.imu.accel.z = 9.81
        m.imu.accel.error = 1 if self.invalid_imu else 0
        m.motor_num = self.motor_count
        for i in range(12):
            m.motor[i].header.limbs_no = i//3
            m.motor[i].header.joint_no = i % 3
            m.motor[i].online = 1
        self.motion.publish(m)
        s = SensorObserved()
        s.timestamp = m.timestamp
        s.odom.valid = 0 if self.invalid_odom else 1
        s.odom.epoch = self.epoch
        s.odom.position[0] = 3. if self.epoch == 1 else 0.
        s.odom.position[1] = 4. if self.epoch == 1 else 0.
        s.odom.yaw = math.pi/2 if self.epoch == 1 else 0.
        self.sensor.publish(s)

    def revoke(self):
        self.owner = ''
        e = EventMessage()
        e.magic = 0x53425645
        e.topic = 'robotServer.control.status'
        e.payload = json.dumps({'controlled': False})
        for _ in range(5):
            self.event.publish(e)
            time.sleep(.02)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    args = parser.parse_args()
    robot_context, business_context = Context(), Context()
    rclpy.init(context=robot_context, domain_id=174)
    rclpy.init(context=business_context, domain_id=175)
    robot_node = rclpy.create_node('cyvet_fixture', context=robot_context)
    node = rclpy.create_node('cyvet_test', context=business_context)
    business_executor = SingleThreadedExecutor(context=business_context)
    business_executor.add_node(node)
    executor = MultiThreadedExecutor(context=robot_context, num_threads=2)
    executor.add_node(robot_node)
    robot = Robot(robot_node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    messages, counts = {}, {}
    subscriptions = []
    for topic, typ in [('/battery/level', UInt8), ('/imu/data', Imu), ('/joint_states', JointState),
                       ('/leg_odom', Odometry), ('/nav_bridge_node/backend_status', String)]:
        def receive(msg, key=topic):
            messages[key] = msg
            counts[key] = counts.get(key, 0) + 1
        subscriptions.append(node.create_subscription(typ, topic, receive,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
    vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = '175'
    log = tempfile.TemporaryFile(mode='w+')
    process = subprocess.Popen([args.executable, '--ros-args', '-p', 'device_id:=cyvet-test',
        '-p', 'robot_domain_id:=174', '-p', 'action_timeout_ms:=4500',
        '-p', 'telemetry_timeout_ms:=800', '-p', 'rpc_timeout_ms:=200'], env=env, stdout=log, stderr=log)

    def wait(predicate, timeout=8.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            business_executor.spin_once(timeout_sec=.03)
            if predicate():
                return
            if process.poll() is not None:
                raise AssertionError(f'bridge exited {process.returncode}')
        raise AssertionError('Condition timed out: ' + str(robot.calls[-15:]) + ' ' + str(messages.get('/nav_bridge_node/backend_status')))

    def service(name, typ=Trigger, req=None):
        client = node.create_client(typ, '/nav_bridge_node/' + name)
        assert client.wait_for_service(timeout_sec=5.), name
        future = client.call_async(req or Trigger.Request())
        wait(future.done, 12.)
        response = future.result()
        if hasattr(response, 'success'):
            print(name, response.success, response.message, flush=True)
        node.destroy_client(client)
        return response

    def velocity(x=.6, y=.4, yaw=.8):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = x, y, yaw
        vel_pub.publish(msg)

    def state():
        return json.loads(messages['/nav_bridge_node/backend_status'].data)

    def set_gait(value):
        v = (ParameterValue(type=2, integer_value=value) if isinstance(value, int)
             else ParameterValue(type=4, string_value=value))
        req = SetParameters.Request(parameters=[Parameter(name='gait', value=v)])
        return service('set_gait', SetParameters, req).results[0]

    try:
        wait(lambda: '/battery/level' in messages and counts.get('/joint_states', 0)>2)
        assert not robot.owner and not any(m == 'takeMotionControl' for m, _ in robot.calls)
        velocity()
        time.sleep(.2)
        assert not robot.owner, 'cmd_vel must not acquire control'
        assert not set_gait('FAST').successful
        assert not robot.owner and not any(m == 'takeMotionControl' for m, _ in robot.calls)
        assert messages['/battery/level'].data == 73
        assert abs(messages['/imu/data'].orientation.w - 1) < 1e-6
        # Flush old packets before checking that invalid samples are suppressed.
        robot.invalid_imu = robot.invalid_odom = True
        for invalid_count in (0, 13):
            robot.motor_count = invalid_count
            settle_until = time.monotonic() + .2
            wait(lambda: time.monotonic() > settle_until)
            saved = {topic: counts.get(topic, 0) for topic in ('/imu/data', '/leg_odom', '/joint_states')}
            settle_until = time.monotonic() + .25
            wait(lambda: time.monotonic() > settle_until)
            assert all(counts.get(topic, 0) == count for topic, count in saved.items())
        robot.invalid_imu = robot.invalid_odom = False
        robot.motor_count = 12
        robot.invalid_quaternion = True
        wait(lambda: messages['/imu/data'].orientation_covariance[0] == -1)
        robot.invalid_quaternion = False
        wait(lambda: messages['/imu/data'].orientation_covariance[0] != -1)
        robot.wrong_identity = True
        assert not service('stand').success
        robot.wrong_identity = False
        robot.reject_acquire = True
        assert not service('stand').success
        robot.reject_acquire = False
        assert service('stand').success
        assert robot.profile == 'slow'
        wait(lambda: state()['navigation_ready'] and state()['gait'] == 0)
        assert not set_gait('MEDIUM').successful
        assert not set_gait(34).successful
        req = SetParameters.Request(parameters=[Parameter(name='speed',
                                    value=ParameterValue(type=2, integer_value=2))])
        assert not service('set_speed', SetParameters, req).results[0].successful
        velocity()
        wait(lambda: robot.velocity[0] != 0.)
        assert set_gait('fast').successful
        wait(lambda: state()['gait'] == 3 and state()['navigation_ready'])
        assert robot.profile == 'fast' and robot.velocity == [0., 0., 0.]
        assert state()['control_profile_acknowledged'] == 'fast'
        assert state()['control_profile_reported'] == ''  # Real firmware may omit it.
        settle_until = time.monotonic() + .6
        wait(lambda: time.monotonic() > settle_until)
        assert robot.velocity == [0., 0., 0.], 'switch must discard cached velocity'
        velocity()
        wait(lambda: robot.velocity[0] != 0.)
        assert robot.velocity == [.6, .4, .8], 'no local speed-tier clipping'
        assert robot.action_params[-1]['controlProfile'] == 'fast'
        assert set_gait(0).successful and robot.profile == 'slow'
        robot.report_profile = True
        robot.profile_lag_ticks = 3  # Firmware ACK precedes effective state updates.
        assert set_gait(3).successful
        wait(lambda: state()['control_profile_reported'] == 'fast')
        robot.profile_lag_ticks = 0
        assert service('ready').success and robot.profile == 'slow', 'ready restores default slow'
        robot.reject_profile = 'fast'
        assert not set_gait('RUN').successful
        wait(lambda: not state()['navigation_ready'])
        assert robot.velocity == [0., 0., 0.] and state()['gait'] == 0
        robot.reject_profile = ''
        assert service('ready').success
        robot.profile_override = 'slow'
        assert not set_gait('FAST').successful, 'reported model mismatch must fail'
        wait(lambda: not state()['navigation_ready'] and robot.velocity == [0., 0., 0.])
        robot.profile_override = ''
        assert service('ready').success
        robot.report_profile = False
        # A queued stop must cancel a profile change, including an ambiguous
        # profile RPC whose reply arrives after its deadline.
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = 'setMotionActionParams', .35, 1
        profile_client = node.create_client(SetParameters, '/nav_bridge_node/set_gait')
        req = SetParameters.Request(parameters=[Parameter(name='gait',
                                    value=ParameterValue(type=4, string_value='FAST'))])
        profile_future = profile_client.call_async(req)
        wait(lambda: robot.delay_remaining == 0)
        assert service('release_control').success
        wait(profile_future.done)
        assert not profile_future.result().results[0].successful
        assert not robot.owner
        wait(lambda: not state()['navigation_ready'] and not state()['command_cached'])
        node.destroy_client(profile_client)
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = '', 0., -1
        assert service('ready').success and robot.profile == 'slow'
        velocity()
        wait(lambda: robot.velocity[0] != 0.)
        assert all(abs(a-b)<1e-6 for a,b in zip(robot.velocity,[.6,.4,.8]))
        wait(lambda: robot.velocity == [0.,0.,0.])
        assert any(m == 'stopMotionAction' for m,_ in robot.calls), 'watchdog must stop action'
        # A failed stop ACK must trigger an emergency fallback, confirmation,
        # and persistent local inhibit rather than retaining an unsafe action.
        emergency_count = sum(m == 'emergencyStopMotion' for m, _ in robot.calls)
        robot.reject_stop_count = 2
        velocity()
        wait(lambda: robot.velocity[0] != 0.)
        wait(lambda: not state()['navigation_ready'] and not state()['stop_pending'] and
             robot.velocity == [0.,0.,0.])
        assert sum(m == 'emergencyStopMotion' for m, _ in robot.calls) > emergency_count
        robot.reject_stop_count = 0
        assert service('ready').success
        assert service('soft_estop').success
        velocity()
        time.sleep(.2)
        assert robot.velocity == [0.,0.,0.]
        assert service('ready').success
        velocity(float('nan'), 0., 0.)
        settle_until = time.monotonic() + .2
        wait(lambda: time.monotonic() > settle_until)
        assert robot.velocity == [0., 0., 0.], 'NaN must never reach the vendor'
        # Only an actual timeout may retry, and only newer live input in an
        # unchanged walking/control session. The retry must use the latest axes.
        recoveries = state()['velocity_rpc_recoveries']
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = 'setMotionActionParams', .35, 1
        velocity(.1, .03, .1)
        wait(lambda: robot.delay_remaining == 0)

        def recovered():
            velocity(-.1, -.03, -.1)
            assert state()['navigation_ready'], 'a confirmed transient retry must retain readiness'
            return state()['velocity_rpc_recoveries'] > recoveries and robot.velocity[0] < 0

        wait(recovered)
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = '', 0., -1
        wait(lambda: robot.velocity == [0., 0., 0.])
        # Explicit rejection and wrong identity must not be treated as timeouts.
        for failure in ('reject_velocity', 'wrong_velocity_identity'):
            assert service('ready').success
            wait(lambda: state()['navigation_ready'] and not state()['command_cached'])
            setattr(robot, failure, True)
            before = sum(m == 'setMotionActionParams' for m, _ in robot.calls)
            velocity()
            wait(lambda: not state()['navigation_ready'] and robot.velocity == [0., 0., 0.])
            assert sum(m == 'setMotionActionParams' for m, _ in robot.calls) == before+1
            setattr(robot, failure, False)
            assert service('ready').success
            wait(lambda: state()['navigation_ready'])
        # A priority stop arriving during a timeout must prevent the retry even
        # if a newer input has already arrived.
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = 'setMotionActionParams', .35, 1
        before = sum(m == 'setMotionActionParams' for m, _ in robot.calls)
        velocity(.1, 0., 0.)
        wait(lambda: robot.delay_remaining == 0)
        velocity(-.1, 0., 0.)
        cancel_client = node.create_client(Trigger, '/nav_bridge_node/release_control')
        cancel_future = cancel_client.call_async(Trigger.Request())
        wait(cancel_future.done)
        assert cancel_future.result().success and not robot.owner
        assert sum(m == 'setMotionActionParams' for m, _ in robot.calls) == before+1
        node.destroy_client(cancel_client)
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = '', 0., -1
        assert service('ready').success
        wait(lambda: state()['navigation_ready'])
        # The first idle lease reply can be late. A short lease must retry well
        # before expiry, while retaining the same controller without reacquiring.
        assert service('release_control').success
        robot.lease_ms = 2000
        assert service('ready').success
        wait(lambda: state()['navigation_ready'])
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = 'renewMotionControl', 3., 1
        acquisitions = sum(m == 'takeMotionControl' for m, _ in robot.calls)
        renewals = sum(m == 'renewMotionControl' for m, _ in robot.calls)
        wait(lambda: sum(m == 'renewMotionControl' for m, _ in robot.calls) >= renewals+2)
        settle_until = time.monotonic()+.4
        wait(lambda: time.monotonic() > settle_until)
        assert state()['navigation_ready'] and state()['control_owned'] and robot.owner
        assert sum(m == 'takeMotionControl' for m, _ in robot.calls) == acquisitions
        robot.delay_method, robot.delay_seconds, robot.delay_remaining = '', 0., -1
        assert service('release_control').success
        robot.lease_ms = 5000
        assert service('ready').success
        wait(lambda: state()['navigation_ready'])
        # Stale motion state must inhibit even when battery/system queries work.
        robot.motion_offline = True
        wait(lambda: not state()['navigation_ready'])
        robot.motion_offline = False
        wait(lambda: not state()['stop_pending'])
        assert service('ready').success
        # A request that times out may take effect later; local inhibit persists.
        robot.delay_method, robot.delay_seconds = 'setMotionActionParams', .35
        velocity()
        wait(lambda: not state()['navigation_ready'])
        robot.delay_method, robot.delay_seconds = '', 0.
        wait(lambda: robot.velocity == [0., 0., 0.])
        assert service('release_control').success and not robot.owner
        robot.reject_action = True
        assert not service('stand').success and not robot.owner
        robot.reject_action = False
        # Stop must preempt a preparation waiting for the vendor master switch.
        client = node.create_client(Trigger, '/nav_bridge_node/stand')
        acquire_count = sum(m == 'setMotionMasterRole' for m, _ in robot.calls)
        future = client.call_async(Trigger.Request())
        wait(lambda: sum(m == 'setMotionMasterRole' for m, _ in robot.calls) > acquire_count)
        release_client = node.create_client(Trigger, '/nav_bridge_node/release_control')
        release_future = release_client.call_async(Trigger.Request())
        settle_until = time.monotonic() + .15
        wait(lambda: time.monotonic() > settle_until)
        assert not service('ready').success, 'ready cannot supersede a pending stop'
        wait(release_future.done)
        assert release_future.result().success
        node.destroy_client(release_client)
        wait(future.done)
        assert not future.result().success and not robot.owner
        node.destroy_client(client)
        assert service('stand').success
        before = messages['/leg_odom'].pose.pose.position
        robot.epoch = 2
        time.sleep(.15)
        wait(lambda: counts.get('/leg_odom',0)>10)
        after = messages['/leg_odom'].pose.pose.position
        assert abs(before.x-after.x)<1e-5 and abs(before.y-after.y)<1e-5
        robot.revoke()
        wait(lambda: not state()['navigation_ready'])
        velocity()
        time.sleep(.2)
        assert not robot.owner
        assert service('stand').success
        robot.offline = True
        wait(lambda: not state()['connected'])
        wait(lambda: not state()['battery_valid'])
        wait(lambda: not robot.owner, 8.)
        robot.offline = False
        wait(lambda: state()['connected'], 12.)
        assert not state()['navigation_ready'] and not robot.owner
        req = SetParameters.Request()
        req.parameters = [Parameter(name='gait', value=ParameterValue(type=2,integer_value=33))]
        assert not service('set_gait',SetParameters,req).results[0].successful
        req.parameters[0].value.integer_value = 0
        assert not service('set_gait',SetParameters,req).results[0].successful
        assert not robot.owner and not state()['navigation_ready']
        assert not service('charge_command',SetParameters,req).results[0].successful
        assert service('lie').success and not robot.owner
        assert service('ready').success
        process.send_signal(signal.SIGTERM)
        process.wait(timeout=15.)
        assert process.returncode == 0 and not robot.owner
        print('PASS: read-only startup, two vendor profiles/no local clipping/default slow, profile rejection/mismatch/cache discard, invalid telemetry/quaternion/NaN, identity rejection, watchdog, motion stale, fresh-input timeout recovery, explicit rejection/wrong identity no retry, expired-input timeout stop, estop/recovery, rejection, concurrent stop, preemption, reconnect, odom continuity, compatibility, shutdown')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        log.seek(0)
        print(log.read()[-5000:])
        log.close()
        executor.shutdown()
        spin.join(timeout=3.)
        business_executor.shutdown()
        node.destroy_node()
        robot_node.destroy_node()
        robot_context.shutdown()
        business_context.shutdown()


if __name__ == '__main__':
    main()
