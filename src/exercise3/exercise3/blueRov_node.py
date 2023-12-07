import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode, SetMode_Request

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn
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

        self.rc_override_pub = self.create_publisher(
            OverrideRCIn, 'mavros/rc/override', 10)

        self.override_timer = None

        self.pitch = 0
        self.roll = 0
        self.throttle = 0
        self.yaw = 0
        self.forward = 0
        self.lateral = 0
        self.camera_pan = 0
        self.camera_tilt = 0
        self.light_level1 = 0
        self.light_level2 = 0
        self.video_switch = 0

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

    def set_mavros_param(self, name, type, value):
        parameter_dict = {
            ParameterType.PARAMETER_BOOL: 'bool_value',
            ParameterType.PARAMETER_INTEGER: 'integer_value',
            ParameterType.PARAMETER_DOUBLE: 'double_value',
            ParameterType.PARAMETER_STRING: 'string_value',
            ParameterType.PARAMETER_BYTE_ARRAY: 'byte_array_value',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array_value',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'integer_array_value',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'double_array_value',
            ParameterType.PARAMETER_STRING_ARRAY: 'string_array_value'
        }

        req = SetParameters.Request()

        parameter = Parameter()
        parameter.name = name
        parameter.value.type = type
        setattr(parameter.value, parameter_dict[type], value)
        req.parameters.append(parameter)

        return self.call_service(
            SetParameters, 'mavros/param/set_parameters', req)

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

    def rc_override_publish_cb(self):
        # convert normalized value(-1 to 1) to a pwm value(1100 to 1900)
        def normalized_to_pwm(x): return int((1900 - 1500) * x + 1500)
        channels = [normalized_to_pwm(self.pitch),
                    normalized_to_pwm(self.roll),
                    normalized_to_pwm(self.throttle),
                    normalized_to_pwm(self.yaw),
                    normalized_to_pwm(self.forward),
                    normalized_to_pwm(self.lateral),
                    normalized_to_pwm(self.camera_pan),
                    normalized_to_pwm(self.camera_tilt),
                    normalized_to_pwm(self.light_level1),
                    normalized_to_pwm(self.light_level2),
                    normalized_to_pwm(self.video_switch),
                    0, 0, 0, 0, 0, 0, 0]

        override_msg = OverrideRCIn()
        override_msg.channels = channels

        self.rc_override_pub.publish(override_msg)

    # arguments should be a normalized float value ranging from -1 to 1
    # 0 is translated to 1500 pwm, -1 to 1100, and 1 to 1900
    def set_rc_override_channels(self, pitch=0, roll=0, throttle=0, yaw=0,
                                 forward=0, lateral=0, camera_pan=0,
                                 camera_tilt=0, light_level1=0, light_level2=0,
                                 video_switch=0):
        self.pitch = pitch
        self.roll = roll
        self.throttle = throttle
        self.yaw = yaw
        self.forward = forward
        self.lateral = lateral
        self.camera_pan = camera_pan
        self.camera_tilt = camera_tilt
        self.light_level1 = light_level1
        self.light_level2 = light_level2
        self.video_switch = video_switch

    def toogle_rc_override(self, run_boolean):
        if run_boolean and self.override_timer is None:
            timer_period = 0.5  # seconds
            self.override_timer = self.create_timer(
                timer_period, self.rc_override_publish_cb)
        elif run_boolean and self.override_timer.is_canceled():
            self.override_timer.reset()
        elif not run_boolean and not self.override_timer.is_canceled():
            self.override_timer.cancel()


def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    # while ardusub.status.mode != "ALT_HOLD":
    #     ardusub.set_mode("ALT_HOLD")
    #     # offb_set_mode = SetMode_Request()
    #     # offb_set_mode.custom_mode = 'OFFBOARD'
    #     service_timer.sleep()
    while ardusub.status.mode != "MANUAL":
        ardusub.set_mode("MANUAL")
        # offb_set_mode = SetMode_Request()
        # offb_set_mode.custom_mode = 'OFFBOARD'
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed == False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    timer = ardusub.create_rate(2)  # Hz

    ardusub.toogle_rc_override(True)

    ardusub.toogle_rc_override(True)
    ardusub.set_rc_override_channels(forward=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(forward=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0)
    # ardusub.toogle_rc_override(False)

    print("Mission completed")


def main(args=None):
    rclpy.init(args=args)

    print("Starting Bluerov agent node")

    ardusub = BlueRovInterface()

    # thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    # thread.start()

    # mission(ardusub)

 

    # # rate = ardusub.create_rate(2)
    # # goal_pose = ardusub.pose_stamped(4.0, 5.0, -2.0, ardusub.local_pos.pose.orientation.x,
    # #                                  ardusub.local_pos.pose.orientation.y, ardusub.local_pos.pose.orientation.z, ardusub.local_pos.pose.orientation.w)
    # # ardusub.setpoint_position_local_pose(goal_pose)

    # # try:
    # #     while rclpy.ok():
    # #         rate.sleep()
    # #         if (ardusub.check_setpoint_reached_xy(goal_pose, 1)):
    # #             while ardusub.status.armed == True:
    # #                 ardusub.arm_motors(False)
    # #                 rate.sleep()
    # #             break
    # # except KeyboardInterrupt:
    # #     pass
    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # # ardusub.arm_motors(False)
    # ardusub.destroy_node()
    # rclpy.shutdown()
    # thread.join()


    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    # TODO:  Known modes are: MANUAL THROW BRAKE POSHOLD AUTOTUNE STABILIZE TRANSECT ACRO FLIP ALT_HOLD AUTO GUIDED VELHOLD RTL CIRCLE SURFACE OF_LOITER DRIFT

    service_timer = ardusub.create_rate(2)
    while ardusub.status.mode != "STABILIZE":
            ardusub.set_mode("STABILIZE")
            service_timer.sleep()
    print("Manual mode selected")

    # TODO: Arm motors
    while ardusub.status.armed == False:
            ardusub.arm_motors(True)
            service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    # TODO: start publishing on /mavros/rc/override
    ardusub.toogle_rc_override(True)
    ardusub.set_rc_override_channels(throttle=0.0, pitch=0.0, forward=0.7)
    # TODO: start moving forward

    # Sonar subscriber

    rate = ardusub.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass
    ardusub.arm_motors(False) # better safe than sorry
    ardusub.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
