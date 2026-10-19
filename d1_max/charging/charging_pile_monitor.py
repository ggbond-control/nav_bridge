#!/usr/bin/env python3
"""Passively display D1 RobotState UDP reports received by the current SDK client.

No SDK connection or control commands. Requires root for AF_PACKET capture.
"""
import argparse
from datetime import datetime
import json
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='eth1')
    parser.add_argument('--host', default='192.168.168.168')
    parser.add_argument('--samples', type=int, default=0, help='0: monitor until Ctrl+C')
    args = parser.parse_args()
    count = 0
    last_report = last_warning = time.monotonic()
    with socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0800)) as capture:
        capture.bind((args.interface, 0))
        capture.settimeout(1)
        print('Passive monitor; charging_pile_connected reads data.charging_pile.connect. Ctrl+C to stop.', flush=True)
        while args.samples <= 0 or count < args.samples:
            now = time.monotonic()
            if now - last_report >= 5 and now - last_warning >= 5:
                print(f'{datetime.now().astimezone().isoformat(timespec="seconds")} '
                      'charging_pile_connected=UNKNOWN (no fresh RobotState for 5 seconds)', flush=True)
                last_warning = now
            try:
                frame = capture.recv(65535)
            except socket.timeout:
                continue
            packet = frame[14:]
            if len(packet) < 28 or packet[0] >> 4 != 4 or packet[9] != 17:
                continue
            if socket.inet_ntoa(packet[12:16]) != args.host:
                continue
            header = (packet[0] & 15) * 4
            if len(packet) < header + 8:
                continue
            source, destination, length, _ = struct.unpack('!HHHH', packet[header:header + 8])
            if source != 8082:
                continue
            payload = packet[header + 8:header + length]
            if not payload.startswith(b'ZSKJ'):
                continue
            try:
                message = json.loads(payload[payload.index(b'{'):])
            except (ValueError, UnicodeDecodeError):
                continue
            if message.get('head', {}).get('type') != 1004:
                continue
            data = message.get('data', {})
            connected = data.get('charging_pile', {}).get('connect')
            if not isinstance(connected, bool):
                continue
            count += 1
            last_report = time.monotonic()
            battery = data.get('battery', {})
            row = {
                'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
                'sample': count,
                'robot_time_ms': message['head'].get('time'),
                'charging_pile_connected': connected,
                'machine_status': data.get('current_state'),
                'motion_status': data.get('motion_status'),
                'battery1_status': battery.get('power_supply_status1'),
                'battery2_status': battery.get('power_supply_status2'),
                'battery1_percent': battery.get('power1'),
                'battery2_percent': battery.get('power2'),
                'battery1_current_A': battery.get('current1'),
                'battery2_current_A': battery.get('current2'),
                'sdk_client_port': destination,
            }
            print(json.dumps(row), flush=True)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nMonitor stopped.', flush=True)
