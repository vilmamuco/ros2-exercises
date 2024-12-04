import rclpy
from rclpy.node import Node
from turtlesim.srv import SetPen
import sys
import time


class ClientNode(Node):

    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(
            SetPen, '/test_namespace/turtle1/set_pen')
        while not self.cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetPen.Request()

    def send_request(self, red, green, blue):
        self.req.r = red
        self.req.g = green
        self.req.b = blue
        self.req.width = 5
        self.req.off = 0
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)


def main():
    rclpy.init()

    red = 255  # pen rgb colors
    green = 0
    blue = 0
    client_node = ClientNode()

    while True:
        # switch between red and green
        red, green = green, red

        response = client_node.send_request(red, green, blue)
        time.sleep(0.2)

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
