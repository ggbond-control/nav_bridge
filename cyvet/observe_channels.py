#!/usr/bin/env python3
"""Passively inspect host-domain observations; no RPCs or control changes."""
import argparse
import json
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from uniubi.msg import MotionObserved, SensorObserved, EventMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=10)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('cyvet_passive_channel_probe')
    types = {'/motion/observed': MotionObserved, '/sensor/observed': SensorObserved,
             '/robotServer/Event': EventMessage}
    counts = dict.fromkeys(types, 0)
    last = {}
    subscriptions = []

    def receive(topic, message):
        counts[topic] += 1
        if topic == '/motion/observed':
            last[topic] = {'timestamp': message.timestamp, 'motor_num': message.motor_num,
                           'battery_percent': message.power.power,
                           'accel_error': message.imu.accel.error,
                           'gyro_error': message.imu.gyro.error}
        elif topic == '/sensor/observed':
            last[topic] = {'timestamp': message.timestamp, 'odom_valid': message.odom.valid,
                           'odom_epoch': message.odom.epoch}
        else:
            last[topic] = {'timestamp': message.timestamp, 'magic': message.magic,
                           'topic': message.topic}

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    try:
        for topic, msg_type in types.items():
            subscriptions.append(node.create_subscription(
                msg_type, topic, lambda msg, topic=topic: receive(topic, msg), qos))
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        for topic in types:
            endpoints = []
            for info in node.get_publishers_info_by_topic(topic):
                endpoints.append({'type': info.topic_type,
                                  'reliability': info.qos_profile.reliability.name,
                                  'durability': info.qos_profile.durability.name,
                                  'depth': info.qos_profile.depth})
            print(json.dumps({'topic': topic, 'publishers': endpoints,
                              'sample_count': counts[topic], 'last_sample': last.get(topic)},
                             ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
