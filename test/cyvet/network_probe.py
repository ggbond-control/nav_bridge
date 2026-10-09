#!/usr/bin/env python3
"""Read-only UniUbi host probe. Never acquires control or enables observations."""
import argparse
import json
import sys
import time

import rclpy
from uniubi.srv import System


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device-id', required=True)
    parser.add_argument('--service', default='/robotServer')
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--repeat', type=int, default=3)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('cyvet_readonly_network_probe')
    try:
        client = node.create_client(System, args.service)
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(json.dumps({'success': False, 'error': 'RPC endpoint not discovered'}))
            return 1
        time.sleep(0.5)  # Allow the reverse DDS response endpoint to match.
        success = True
        for iteration in range(args.repeat):
            for method in ('getMotionCapabilities', 'getSystemStatus', 'queryMotionState', 'getMotorLayout'):
                request = System.Request()
                request.timestamp = time.time_ns() // 1_000_000
                request.device_id = args.device_id
                request.service = 'robotAppService'
                request.method = method
                request.payload = json.dumps({
                    'call': {'clientId': 'cyvet-readonly-probe'}, 'params': {}})
                future = client.call_async(request)
                rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
                record = {'iteration': iteration + 1, 'method': method}
                if not future.done():
                    client.remove_pending_request(future)
                    record.update(success=False, error='RPC response timeout')
                elif future.exception():
                    record.update(success=False, error=str(future.exception()))
                else:
                    response = future.result()
                    try:
                        payload = json.loads(response.payload)
                        valid = (response.code == 0 and payload.get('result') is True
                                 and response.device_id == args.device_id)
                        record.update(success=valid, code=response.code,
                                      device_id=response.device_id, payload=payload)
                    except (ValueError, AttributeError) as exc:
                        record.update(success=False, error=str(exc))
                print(json.dumps(record, ensure_ascii=False), flush=True)
                success &= record['success']
        return 0 if success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
