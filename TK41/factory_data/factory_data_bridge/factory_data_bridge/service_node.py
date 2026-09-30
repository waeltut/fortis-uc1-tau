import rclpy
from rclpy.node import Node
from factory_data_interfaces.srv import GetFactoryData
from factory_data_bridge.http_client import fetch_data


class FactoryDataService(Node):
    def __init__(self):
        super().__init__('factory_data_service')
        self.declare_parameter('server_url', 'http://127.0.0.1:8000/data')
        self.declare_parameter('timeout_sec', 60.0)
        self.declare_parameter('max_response_bytes', 52428800)
        self.service = self.create_service(GetFactoryData, 'get_factory_data', self.handle_request)
        self.get_logger().info('Ready: get_factory_data -> ' + self.get_parameter('server_url').value)

    def handle_request(self, request, response):
        del request
        response.success = False
        response.json_data = ''
        try:
            text, data = fetch_data(
                self.get_parameter('server_url').value,
                self.get_parameter('timeout_sec').value,
                self.get_parameter('max_response_bytes').value)
            # Partial or empty datasets remain valid responses; skipped files are explicit.
            response.success = True
            response.json_data = text
            response.message = (f"Loaded {data['file_count']} files, {data['row_count']} rows; "
                                f"skipped {data['skipped_count']} files")
            if data['skipped_count']:
                self.get_logger().warning(response.message)
            else:
                self.get_logger().info(response.message)
        except Exception as error:
            response.message = f'Factory data request failed: {error}'
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FactoryDataService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
