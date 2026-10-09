#!/usr/bin/env python3
"""Explicit, bounded physical Cyvet checks. Requires an attended clear test area."""
import argparse
import json
import math
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import Imu, JointState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stage', required=True, choices=['stand', 'axes', 'combined', 'takeover', 'estop', 'lie', 'zero_stream', 'idle_hold', 'profiles'])
    parser.add_argument('--executable', help='Optionally launch a fresh node for this one check')
    parser.add_argument('--params', help='Required when --executable is used')
    parser.add_argument('--seconds', type=float, default=1., help='Per-axis duration, 0.1 to 5 seconds')
    parser.add_argument('--axes', choices=['all', 'lateral', 'yaw', 'lateral_yaw'], default='all')
    parser.add_argument('--lateral-speed', type=float, default=.06)
    parser.add_argument('--yaw-speed', type=float, default=.1)
    parser.add_argument('--hold-seconds', type=float, default=10., help='Zero/idle hold duration, 1 to 180 seconds')
    args = parser.parse_args()
    if not 0 < args.lateral_speed <= .1 or not 0 < args.yaw_speed <= .3:
        parser.error('Attended checks require lateral speed <=0.1 m/s and yaw <=0.3 rad/s')
    if not .1 <= args.seconds <= 5.:
        parser.error('--seconds must be between 0.1 and 5')
    if not 1 <= args.hold_seconds <= 180:
        parser.error('--hold-seconds must be between 1 and 180')
    if args.executable and not args.params:
        parser.error('--executable requires --params')
    rclpy.init()
    node = rclpy.create_node('cyvet_attended_motion_check')
    status = {}

    def receive(message):
        status.clear()
        status.update(json.loads(message.data))

    sub = node.create_subscription(String, '/nav_bridge_node/backend_status', receive, 10)
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    counts, telemetry, data_subscriptions = {}, {}, []
    phase_gyro = []
    phase_active = False
    odom_metrics = {"max_step_m": 0., "max_yaw_step_rad": 0.}
    for topic, typ in [('/imu/data', Imu), ('/joint_states', JointState), ('/leg_odom', Odometry)]:
        def sample(message, topic=topic):
            counts[topic] = counts.get(topic, 0) + 1
            if topic == '/joint_states':
                telemetry[topic] = {'names': list(message.name), 'positions': list(message.position)}
            elif topic == '/leg_odom':
                previous = telemetry.get(topic)
                x, y = message.pose.pose.position.x, message.pose.pose.position.y
                qz, qw = message.pose.pose.orientation.z, message.pose.pose.orientation.w
                if previous:
                    odom_metrics['max_step_m'] = max(odom_metrics['max_step_m'],
                        math.hypot(x-previous['x'], y-previous['y']))
                    dyaw = 2*math.atan2(qz,qw)-2*math.atan2(previous['qz'],previous['qw'])
                    odom_metrics['max_yaw_step_rad'] = max(odom_metrics['max_yaw_step_rad'],
                        abs(math.atan2(math.sin(dyaw),math.cos(dyaw))))
                telemetry[topic] = {'x': message.pose.pose.position.x, 'y': message.pose.pose.position.y,
                                    'qz': message.pose.pose.orientation.z, 'qw': message.pose.pose.orientation.w}
            else:
                if phase_active:
                    phase_gyro.append(message.angular_velocity.z)
                telemetry[topic] = {'accel_z': message.linear_acceleration.z,
                                    'gyro_z': message.angular_velocity.z}
        data_subscriptions.append(node.create_subscription(typ, topic, sample,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)))
    clients = {}
    process = (subprocess.Popen([args.executable, '--ros-args', '--params-file', args.params])
               if args.executable else None)

    def wait(predicate, timeout=20.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.025)
            if predicate():
                return
        raise RuntimeError('Condition timed out: ' + json.dumps(status))

    def call(name):
        if name not in clients:
            clients[name] = node.create_client(Trigger, '/nav_bridge_node/' + name)
        client = clients[name]
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('Missing service: ' + name)
        future = client.call_async(Trigger.Request())
        wait(future.done)
        response = future.result()
        print(json.dumps({'service': name, 'success': response.success,
                          'message': response.message, 'status': status}), flush=True)
        if not response.success:
            raise RuntimeError(name + ': ' + response.message)

    def pulse(label, x, y, yaw):
        nonlocal phase_active
        phase_gyro.clear()
        phase_active = True
        initial_odom = telemetry.get('/leg_odom')
        print(json.dumps({'pulse': label, 'vx': x, 'vy': y, 'yaw': yaw, 'seconds': args.seconds}), flush=True)
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = x, y, yaw
        end = time.monotonic() + args.seconds
        reported = False
        while time.monotonic() < end:
            if not status.get('navigation_ready') or not status.get('control_owned'):
                raise RuntimeError('Readiness/control lost during pulse')
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=.04)
            reported |= all(abs(status.get(k, float('inf'))-v)<.005
                            for k,v in zip(('vx','vy','yaw_rate'),(x,y,yaw)))
        phase_active = False
        # Stop publishing deliberately tests the bridge input watchdog.
        settled = time.monotonic() + .7
        wait(lambda: time.monotonic() > settled and
             all(abs(status.get(k, 1.)) < .001 for k in ('vx', 'vy', 'yaw_rate')), 5.)
        if not status.get('navigation_ready'):
            raise RuntimeError('Pulse stopped after a fault: ' + json.dumps(status))
        movement = {}
        final_odom = telemetry.get('/leg_odom')
        if initial_odom and final_odom:
            start_yaw = 2*math.atan2(initial_odom['qz'],initial_odom['qw'])
            end_yaw = 2*math.atan2(final_odom['qz'],final_odom['qw'])
            dx,dy = final_odom['x']-initial_odom['x'],final_odom['y']-initial_odom['y']
            movement = {'body_dx': math.cos(start_yaw)*dx+math.sin(start_yaw)*dy,
                        'body_dy': -math.sin(start_yaw)*dx+math.cos(start_yaw)*dy,
                        'dyaw': math.atan2(math.sin(end_yaw-start_yaw),math.cos(end_yaw-start_yaw))}
        print(json.dumps({'pulse_stopped': label, 'reported_motion': reported, 'status': status,
                          'odom_delta': movement, 'imu_gyro_z_mean':
                          sum(phase_gyro)/len(phase_gyro) if phase_gyro else None}), flush=True)
        if not reported:
            raise RuntimeError('Robot did not report a nonzero control velocity')

    def select_profile(profile, gait):
        client = node.create_client(SetParameters, '/nav_bridge_node/set_gait')
        try:
            if not client.wait_for_service(timeout_sec=3.):
                raise RuntimeError('Missing set_gait service')
            request = SetParameters.Request(parameters=[Parameter(name='gait',
                value=ParameterValue(type=4, string_value=profile))])
            future = client.call_async(request)
            wait(future.done)
            result = future.result().results[0]
            print(json.dumps({'service': 'set_gait', 'profile': profile,
                'success': result.successful, 'message': result.reason}), flush=True)
            if not result.successful:
                raise RuntimeError('Profile switch failed: ' + result.reason)
            wait(lambda: status.get('gait') == gait and status.get('navigation_ready') and
                 status.get('control_profile_acknowledged') == profile and
                 not status.get('command_cached') and
                 all(abs(status.get(k, 1.)) < .001 for k in ('vx', 'vy', 'yaw_rate')))
        finally:
            node.destroy_client(client)

    try:
        wait(lambda: status.get('connected') and status.get('battery_valid'))
        if status.get('control_owned') or status.get('navigation_ready'):
            raise RuntimeError('Expected a fresh read-only node')
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        if any(p.node_name != node.get_name() for p in publishers):
            raise RuntimeError('Another cmd_vel publisher is present; isolate the test domain')
        call('stand')
        wait(lambda: status.get('navigation_ready') and status.get('control_owned'))
        if args.stage == 'axes':
            pulses = []
            if args.axes == 'all':
                pulses += [('forward', .1, 0., 0.), ('backward', -.1, 0., 0.)]
            if args.axes in ('all','lateral','lateral_yaw'):
                pulses += [('left', 0., args.lateral_speed, 0.), ('right', 0., -args.lateral_speed, 0.)]
            if args.axes in ('all','yaw','lateral_yaw'):
                pulses += [('left_turn', 0., 0., args.yaw_speed), ('right_turn', 0., 0., -args.yaw_speed)]
            for values in pulses:
                pulse(*values)
        elif args.stage == 'combined':
            # Keep the previously calibrated low test velocities.
            pulse('forward_left_left_turn', .1, args.lateral_speed, args.yaw_speed)
            pulse('backward_right_right_turn', -.1, -args.lateral_speed, -args.yaw_speed)
        elif args.stage == 'profiles':
            if status.get('control_profile_acknowledged') != 'slow':
                raise RuntimeError('Default deployment must start with slow')
            # Zero axes only: this stage does not publish translational/turning commands.
            for profile, gait in [('fast', 3), ('slow', 0), ('fast', 3)]:
                select_profile(profile, gait)
                settled = time.monotonic() + 2.
                wait(lambda: time.monotonic() > settled)
                if not status.get('navigation_ready') or not status.get('control_owned'):
                    raise RuntimeError('Profile hold lost readiness: ' + json.dumps(status))
            call('ready')
            wait(lambda: status.get('control_profile_acknowledged') == 'slow' and status.get('gait') == 0)
            call('lie')
            wait(lambda: not status.get('control_owned') and status.get('action') == 'laying')
            print('PASS: zero-speed profile RPCs, cache clearing, default slow and laying/release confirmed', flush=True)
            return
        elif args.stage == 'takeover':
            print('TAKEOVER NOW: zero-speed walking; use the remote to take control within 60 seconds',
                  flush=True)
            wait(lambda: not status.get('control_owned'), 60.)
            if status.get('navigation_ready') or status.get('command_cached'):
                raise RuntimeError('Revocation did not clear readiness and velocity')
            # The operator owns the robot now. Never call ready or send nonzero motion.
            end = time.monotonic() + 5.
            while time.monotonic() < end:
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.04)
                if status.get('control_owned') or status.get('navigation_ready'):
                    raise RuntimeError('Bridge reacquired control without explicit ready')
            print('PASS: remote takeover revoked bridge control; zero cmd_vel did not reacquire',
                  flush=True)
            return
        elif args.stage in ('zero_stream', 'idle_hold'):
            end = time.monotonic() + args.hold_seconds
            print(json.dumps({'hold_stage': args.stage, 'seconds': args.hold_seconds}), flush=True)
            while time.monotonic() < end:
                if not status.get('navigation_ready') or not status.get('control_owned'):
                    raise RuntimeError('Zero/idle hold lost readiness or ownership: ' + json.dumps(status))
                if args.stage == 'zero_stream':
                    pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.04)
            settled = time.monotonic() + .7
            wait(lambda: time.monotonic() > settled)
        elif args.stage == 'estop':
            call('soft_estop')
            wait(lambda: not status.get('navigation_ready'))
            for _ in range(5):
                message = Twist()
                message.linear.x = .03
                pub.publish(message)
                rclpy.spin_once(node, timeout_sec=.1)
            if any(abs(status.get(k, 1.)) > .001 for k in ('vx', 'vy', 'yaw_rate')):
                raise RuntimeError('Motion reported while locally emergency-stopped')
            # Firmware emergency-stop cooling period is several seconds.
            cooled = time.monotonic() + 5.
            wait(lambda: time.monotonic() > cooled, 7.)
            call('ready')
        elif args.stage == 'lie':
            call('lie')
            wait(lambda: not status.get('control_owned'))
            print('PASS: laying and release confirmed', flush=True)
            return
        call('release_control')
        wait(lambda: not status.get('control_owned') and not status.get('navigation_ready'))
        print('PASS: ' + args.stage + ' protocol/state checks; physical direction requires observer confirmation', flush=True)
    finally:
        # Always attempt confirmed stopping and release after partial failures.
        if status.get('control_owned'):
            try:
                call('release_control')
                wait(lambda: not status.get('control_owned'))
            except Exception as error:
                print('STOP/RELEASE FAILED: ' + str(error), flush=True)
                try:
                    call('soft_estop')
                except Exception as stop_error:
                    print('REMOTE ESTOP FAILED: ' + str(stop_error), flush=True)
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGTERM)
            try:
                process.wait(timeout=20.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                print('NODE CLEANUP TIMEOUT; verify robot stopped on site', flush=True)
        print(json.dumps({'telemetry_counts': counts, 'last_samples': telemetry, 'odom_metrics': odom_metrics, 'final_status': status}), flush=True)
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
