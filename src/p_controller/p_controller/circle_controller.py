import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from p_interfaces.srv import ChangeRadius


class CircleController(Node):
    def __init__(self):
        super().__init__('circle_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.srv = self.create_service(
            ChangeRadius, 'change_radius', self.change_radius_callback)
        self.radius = 1.0

        self.declare_parameter('angular_z_velocity', 1.0)

    def timer_callback(self):
        # if the angular velocity param is bigger than 1.0, reset it to 1.0
        if self.get_parameter('angular_z_velocity').value > 1.0:
            new_param = rclpy.parameter.Parameter(
                'angular_z_velocity', rclpy.Parameter.Type.DOUBLE, 1.0)
            self.set_parameters([new_param])

        vel = Twist()
        vel.linear.x = self.radius * 1.0  # linear velocity = radius * angular velocity
        vel.angular.z = self.get_parameter('angular_z_velocity').value

        self.publisher_.publish(vel)

    def change_radius_callback(self, request, response):
        self.radius = request.radius
        response.linear_velocity.x = self.radius * 1.0
        response.linear_velocity.y = 0.0
        response.linear_velocity.z = 1.0
        response.radius_changed = True
        return response


def main(args=None):
    rclpy.init(args=args)

    circleController = CircleController()

    rclpy.spin(circleController)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    circleController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
