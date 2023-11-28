import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ControllerSub(Node):
    def __init__(self):
        super().__init__('controller_sub')
        self.subscription = self.create_subscription(
            Twist,
            'turtle1/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('x linear velocity  = "%s"' % msg.linear.x)
        self.get_logger().info('z angular velocity = "%s"' % msg.angular.z)


def main(args=None):
    rclpy.init(args=args)

    controller_sub = ControllerSub()

    rclpy.spin(controller_sub)

    controller_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
