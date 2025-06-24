#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
import rclpy.time
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist
from asv_arov_interfaces.srv import GetOdom
from asv_arov_navigation.utils import build_transform_stamped

class PosePublisher(LifecycleNode):
    def __init__(self):
        super().__init__('pose_publisher')
        
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/amcl_pose')
        self.declare_parameter('use_sim', False)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.odom_topic = self.get_namespace() + self.get_parameter('odom_topic').get_parameter_value().string_value
        self.pose_topic = self.get_namespace() + self.get_parameter('pose_topic').get_parameter_value().string_value
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        if self.use_sim :
            self.cmd_vel_subscriber = self.create_subscription(Twist, self.get_namespace() + "/cmd_vel", self.cmd_vel_callback, 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self._lifecycle_state_publisher = self.create_lifecycle_publisher(
            LifecycleState, self.get_namespace() + '/pose_publisher/state', 10
        )

        # Transient Local QoS profile
        self.amcl_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = self.y = self.z = self.yaw = 0.0
        self.current_velocity = Twist()
        self.odom = Odometry()

        self.timer = None
        self.last_time = None

        x1 = 0.0
        y1 = -3.0
        theta1 = math.pi / 4
        x2 = 0.0
        y2 = 3.0
        theta2 = -math.pi / 4
        w = 1.8288

        r1 = R.from_euler('z', -theta1)
        p1 = r1.apply(np.array([[x1 + w * math.cos(theta1)], [y1 + w * math.sin(theta1)]], dtype=np.float64))
        r2 = R.from_euler('z', -(theta1 + math.pi / 2))
        p2 = r2.apply(np.array([[x1], [y1]], dtype=np.float64))
        r3 = R.from_euler('z', -theta2)
        p3 = r3.apply(np.array([[x2 + w * math.cos(theta2)], [y2 + w * math.sin(theta2)]], dtype=np.float64))
        r4 = R.from_euler('z', -(theta2 - math.pi / 2))
        p4 = r4.apply(np.array([[x2], [y2]], dtype=np.float64))
        self.fence1_transform = [p1[0], p1[1], 0, 0, 0, theta1]
        self.fence2_transform = [p2[0], p2[1], 0, 0, 0, theta1 + math.pi / 2]
        self.fence3_transform = [p3[0], p3[1], 0, 0, 0, theta2]
        self.fence4_transform = [p4[0], p4[1], 0, 0, 0, theta2 - math.pi / 2]

    def cmd_vel_callback(self, data):
        now = self.get_clock().now()
        if self.last_time is not None :
            dt = (self.get_clock().now() - self.last_time).to_msg().nanosec / 1e9
        else :
            dt = 0
        self.last_time = now
        if self.get_namespace() == "/arov" :
            self.x += (data.linear.x * math.cos(self.yaw) + data.linear.y * math.sin(self.yaw)) * dt
            self.y += (data.linear.x * math.sin(self.yaw) + data.linear.y * math.cos(self.yaw)) * dt
            self.z += data.linear.z * dt
            self.yaw += data.angular.z * dt
        elif self.get_namespace() == "/asv" :
            self.x += data.linear.x * math.cos(self.yaw) * dt
            self.y += data.linear.x * math.sin(self.yaw) * dt
            self.z += data.linear.z * dt
            self.yaw += data.angular.z * dt
    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.velocity_callback, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, self.amcl_qos)
        self.odom_srv = self.create_service(GetOdom, self.get_namespace() + '/get_odom', self.odom_srv_callback)
        if not self.use_sim :
            self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odometry_callback, 10)

        else :
            self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        
        msg = LifecycleState()
        msg.id = LifecycleState.PRIMARY_STATE_INACTIVE
        msg.label = 'inactive'
        self._lifecycle_state_publisher.publish(msg)
        # self.last_time = self.get_clock().now()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Activating...')

        self.timer = self.create_timer(0.05, self.update_odometry)

        msg = LifecycleState()
        msg.id = LifecycleState.PRIMARY_STATE_ACTIVE
        msg.label = 'active'
        self._lifecycle_state_publisher.publish(msg)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Deactivating...')
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Cleaning up...')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def velocity_callback(self, msg):
        self.current_velocity = msg

    def update_odometry(self):
        if self.use_sim :

            now = self.get_clock().now()
            #self.last_time = now

            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = 'map'
            tf.child_frame_id = self.get_namespace().strip('/') + '/odom'
            self.tf_broadcaster.sendTransform(tf)

            self.odom.header.stamp = now.to_msg()
            self.odom.header.frame_id = self.get_namespace().strip('/') + '/odom'
            self.odom.child_frame_id = self.get_namespace().strip('/') + '/base_link'

            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.position.z = self.z
            quat = R.from_euler('z', self.yaw).as_quat()
            self.odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            self.odom.twist.twist = self.current_velocity
            self.odom_pub.publish(self.odom)

            tf = TransformStamped()
            tf.header.stamp = self.odom.header.stamp
            tf.header.frame_id = self.get_namespace().strip('/') + '/odom'
            tf.child_frame_id = self.get_namespace().strip('/') + '/base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = self.z
            tf.transform.rotation = self.odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

            if self.get_namespace().strip('/') == "arov" :
                #self.get_logger().info(f'{self.x1 + self.w * math.sin(self.theta1)} {self.x1} {self.x2 + self.w * math.sin(self.theta2)} {self.x2}')
                self.tf_broadcaster.sendTransform(build_transform_stamped(self.get_clock().now(), "map", "fence_1", self.fence1_transform))
                self.tf_broadcaster.sendTransform(build_transform_stamped(self.get_clock().now(), "map", "fence_2", self.fence2_transform))
                self.tf_broadcaster.sendTransform(build_transform_stamped(self.get_clock().now(), "map", "fence_3", self.fence3_transform))
                self.tf_broadcaster.sendTransform(build_transform_stamped(self.get_clock().now(), "map", "fence_4", self.fence4_transform))

            # Simulated amcl_pose
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = self.odom.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.pose = self.odom.pose.pose
            pose.pose.covariance = [
                0.05, 0, 0, 0, 0, 0,
                0, 0.05, 0, 0, 0, 0,
                0, 0, 0.01, 0, 0, 0,
                0, 0, 0, 0.01, 0, 0,
                0, 0, 0, 0, 0.01, 0,
                0, 0, 0, 0, 0, 0.1
            ]
            self.pose_pub.publish(pose)
        else :
            t = None
            try :
                t = self.tf_buffer.lookup_transform(self.get_namespace().strip('/') + '/base_link', 'map', rclpy.time.Time()) # self.get_clock().now()
            except TransformException as ex :
                self.get_logger().info(f'Could not get robot pose as transform: {ex}')
            if t is not None :
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position.x = t.transform.translation.x
                pose.pose.pose.position.y = t.transform.translation.y
                pose.pose.pose.position.z = t.transform.translation.z
                pose.pose.pose.orientation = t.transform.rotation
                pose.pose.covariance = [
                    0.05, 0, 0, 0, 0, 0,
                    0, 0.05, 0, 0, 0, 0,
                    0, 0, 0.01, 0, 0, 0,
                    0, 0, 0, 0.01, 0, 0,
                    0, 0, 0, 0, 0.01, 0,
                    0, 0, 0, 0, 0, 0.1
                ]
                self.pose_pub.publish(pose)
    
    def odometry_callback(self, msg: Odometry) :
        self.odom = msg
    
    def odom_srv_callback(self, request: GetOdom.Request, response: GetOdom.Response) :
        response.odom = self.odom
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
