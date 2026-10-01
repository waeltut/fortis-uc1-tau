#!/usr/bin/env python3
"""Call the service and parse JSON without printing the entire dataset."""
import argparse
import json
import rclpy
from factory_data_interfaces.srv import GetFactoryData


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for field in ('worker', 'station', 'shift', 'date'):
        parser.add_argument('--' + field, default='')
    for field in ('day', 'month', 'year'):
        parser.add_argument('--' + field, type=int, default=0)
    options = parser.parse_args()
    if any(getattr(options, key) < 0 for key in ('day', 'month', 'year')):
        parser.error('day, month and year must be non-negative')
    rclpy.init(args=[])
    node = rclpy.create_node('factory_data_example_client')
    try:
        client = node.create_client(GetFactoryData, '/get_factory_data')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('get_factory_data is not available')
        request = GetFactoryData.Request()
        for key, value in vars(options).items():
            setattr(request, key, value)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=130.0)
        if not future.done():
            raise TimeoutError('ROS service did not respond within 130 seconds')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        data = json.loads(response.json_data)
        print(response.message)
        for entry in data['files']:
            print(entry['relative_path'], 'rows:', entry['row_count'])
        for error in data['errors']:
            print('SKIPPED:', error['relative_path'], error['message'])
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
