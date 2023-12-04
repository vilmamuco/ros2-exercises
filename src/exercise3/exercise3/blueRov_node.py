import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import PoseStamped
import threading


class BlueRovInterface(Node):
    def __init__(self, node_name='bluerov_interface'):
        super().__init__(node_name)
        self.status = State()
        self.state_sub = self.create_subscription(
            State, 'mavros/state', self.status_cb, 10)

        # TODO: this should be optional
        self.local_pos = PoseStamped()
        self.local_pos_received = False
        local_position_sub_qos = QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, depth=5)
        self.local_position_sub = self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.local_pos_cb,
            local_position_sub_qos)

        self.override_timer = None


    def status_cb(self, msg):
        self.status = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg
        self.local_pos_received = True
        print(self.local_pos.pose.position.x)

    def call_service(self, srv_type, srv_name, request):
        service = self.create_client(srv_type, srv_name)
        while not service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(srv_name + ' not available, waiting...')
        future = service.call_async(request)
        return future

    def set_mode(self, mode):
        req = SetMode.Request()
        req.custom_mode = mode
        self.call_service(SetMode, 'mavros/set_mode', req)

    def arm_motors(self, arm_motors_bool):
        req = CommandBool.Request()
        req.value = arm_motors_bool
        self.call_service(CommandBool, 'mavros/cmd/arming', req)

    def setpoint_position_local(
            self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0):

        self.setpoint_poisition_local_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        pose = self.pose_stamped(x, y, z, rx, ry, rz, rw)

        self.setpoint_poisition_local_pub.publish(pose)
        return pose

    def setpoint_position_local_pose(self, pose):
        self.setpoint_poisition_local_pub = self.create_publisher(
            PoseStamped, 'mavros/setpoint_position/local', 10)

        self.setpoint_poisition_local_pub.publish(pose)

    def pose_stamped(
            self, x=.0, y=.0, z=.0, rx=.0, ry=.0, rz=.0, rw=1.0):

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = rx
        pose.pose.orientation.y = ry
        pose.pose.orientation.z = rz
        pose.pose.orientation.w = rw

        return pose

    def check_setpoint_reached(self, pose, delta=0.1):
        return abs(
            self.local_pos.pose.position.x -
            pose.pose.position.x) <= delta and abs(
            self.local_pos.pose.position.y -
            pose.pose.position.y) <= delta and abs(
            self.local_pos.pose.position.z -
            pose.pose.position.z) <= delta

    def check_setpoint_reached_xy(self, pose, delta=0.1):
        return abs(
            self.local_pos.pose.position.x -
            pose.pose.position.x) <= delta and abs(
            self.local_pos.pose.position.y -
            pose.pose.position.y) <= delta


def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    # other modes ex: "ALT_HOLD", "OFFBOARD", "MANUAL" etc.
    while ardusub.status.mode != "GUIDED":
        ardusub.set_mode("GUIDED")
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed == False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    timer = ardusub.create_rate(2)  # Hz

    print("Mission completed")


def main(args=None):
    rclpy.init(args=args)

    print("Starting Bluerov agent node")

    ardusub = BlueRovInterface()

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)

    rate = ardusub.create_rate(2)
    goal_pose = ardusub.pose_stamped(5.0, 0.0, -2.0, ardusub.local_pos.pose.orientation.x,
                                     ardusub.local_pos.pose.orientation.y, ardusub.local_pos.pose.orientation.z, ardusub.local_pos.pose.orientation.w)
    ardusub.setpoint_position_local_pose(goal_pose)

    try:
        while rclpy.ok():
            rate.sleep()
            if (ardusub.check_setpoint_reached_xy(goal_pose, 1)):
                while ardusub.status.armed == True:
                    ardusub.arm_motors(False)
                    rate.sleep()
                break
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # ardusub.arm_motors(False)
    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
