"""Call the high-level dump trash service once and exit."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from robot_interfaces.srv import DumpTrash


class DumpTrashClient(Node):
    """One-shot client for robot_interfaces/srv/DumpTrash."""

    def __init__(self):
        super().__init__('dump_trash_client')
        self.declare_parameter('bucket_id', 1)
        self.declare_parameter('service_name', 'dump_trash')
        self.declare_parameter('timeout_sec', 60.0)

    def call(self, bucket_id, service_name, timeout_sec):
        """Call the service and return True on success."""
        client = self.create_client(DumpTrash, service_name)
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f'service {service_name} not available after '
                f'{timeout_sec:.1f}s'
            )
            return False

        request = DumpTrash.Request()
        request.bucket_id = int(bucket_id)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=timeout_sec,
        )

        if not future.done():
            self.get_logger().error(
                f'service {service_name} call timed out after '
                f'{timeout_sec:.1f}s'
            )
            return False

        response = future.result()
        if response is None:
            self.get_logger().error(
                f'service {service_name} call failed: {future.exception()}'
            )
            return False

        if response.success:
            self.get_logger().info(response.message)
            return True

        self.get_logger().error(response.message)
        return False


def parse_args(argv):
    """Parse non-ROS command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Call the dump trash service once.',
    )
    parser.add_argument('--bucket-id', type=int, choices=(1, 2))
    parser.add_argument('--service-name')
    parser.add_argument('--timeout-sec', type=float)
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main(args=None):
    """Entry point for ros2 run."""
    cli_args = parse_args(args)
    rclpy.init(args=args)
    node = DumpTrashClient()

    try:
        bucket_id = (
            cli_args.bucket_id
            if cli_args.bucket_id is not None
            else node.get_parameter('bucket_id').value
        )
        service_name = (
            cli_args.service_name
            if cli_args.service_name is not None
            else node.get_parameter('service_name').value
        )
        timeout_sec = (
            cli_args.timeout_sec
            if cli_args.timeout_sec is not None
            else node.get_parameter('timeout_sec').value
        )

        ok = node.call(
            bucket_id=int(bucket_id),
            service_name=str(service_name),
            timeout_sec=float(timeout_sec),
        )
        return 0 if ok else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
