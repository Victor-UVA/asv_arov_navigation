import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from robot_guidance_interfaces.action import GoToDepth
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class DepthControlServer(Node) :
    def __init__(self):
        super().__init__('depth_control_server')

        self.tolerance = 0.1
        self.kP = 0.1
        self.max_velocity = 3
        self.timeout = 5
        # Declare topics with defaults
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        # Publishers and Subcribers
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        # Action Server
        self._action_server = ActionServer(
            self,
            GoToDepth,
            'go_to_depth',
            self.goal_callback
        )
        # TF Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_z = None

    def goal_callback(self, msg) :
        start_time = self.get_clock().now()
        self.goal_z = msg.request.target_depth
        while abs(self.error()) > self.tolerance :
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > self.timeout :
                result = GoToDepth.Result()
                result.reached_final_depth = False
                return result
            cmd = Twist()
            cmd.linear.z = self.kP * self.error()
            self.cmd_pub.publish(cmd)
        result = GoToDepth.Result()
        result.reached_final_depth = True
        return result

    def error(self) :
        return self.goal_z - self.get_z()

    def get_z(self) :
        while True :
            try :
                transform = self.tf_buffer.lookup_transform('map', 'arov/base_link', rclpy.time.Time())
                return transform.transform.translation.z
            except TransformException as pose_ex:
                self.get_logger().warning(f'Could not get AROV pose: {pose_ex}')
            