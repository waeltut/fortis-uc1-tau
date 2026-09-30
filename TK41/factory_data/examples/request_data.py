#!/usr/bin/env python3
"""Call the service and parse JSON without printing the entire dataset."""
import json
import rclpy
from factory_data_interfaces.srv import GetFactoryData


def main():
    rclpy.init()
    node = rclpy.create_node('factory_data_example_client')
    try:
        client = node.create_client(GetFactoryData, '/get_factory_data')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('get_factory_data is not available')
        future = client.call_async(GetFactoryData.Request())
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
