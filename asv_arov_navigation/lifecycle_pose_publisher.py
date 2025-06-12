#!/usr/bin/env python3
import math
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
from lifecycle_msgs.msg import State as LifecycleState
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

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
        self.last_time = None
        self.current_velocity = Twist()

        self.timer = None

    
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('Configuring...')

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.velocity_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, self.amcl_qos)
        
        msg = LifecycleState()
        msg.id = LifecycleState.PRIMARY_STATE_INACTIVE
        msg.label = 'inactive'
        self._lifecycle_state_publisher.publish(msg)
        self.last_time = self.get_clock().now()
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
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            vx = self.current_velocity.linear.x
            vy = self.current_velocity.linear.y
            vz = self.current_velocity.linear.z
            vyaw = self.current_velocity.angular.z

            dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            dz = vz * dt
            dyaw = vyaw * dt

            self.x += dx
            self.y += dy
            self.z += dz
            self.yaw += dyaw
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

            odom = Odometry()
            sec, nsec = now.seconds_nanoseconds()
            odom.header.stamp = Time(sec=sec, nanosec=nsec)
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = self.z
            quat = R.from_euler('z', self.yaw).as_quat()
            odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
            odom.twist.twist = self.current_velocity
            self.odom_pub.publish(odom)

            tf = TransformStamped()
            tf.header.stamp = odom.header.stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id = 'base_link'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = self.z
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

            # Simulated amcl_pose
            pose = PoseWithCovarianceStamped()
            pose.header.stamp = odom.header.stamp
            pose.header.frame_id = 'map'
            pose.pose.pose = odom.pose.pose
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
                t = self.tf_buffer.lookup_transform(self.get_namespace(), 'map', self.get_clock().now())
            except TransformException as ex :
                self.get_logger().info(f'Could not get robot pose as transform: {ex}')
            if t is not None :
                pose = PoseWithCovarianceStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.pose.position.x = t.translation.x
                pose.pose.pose.position.y = t.translation.y
                pose.pose.pose.position.z = t.translation.z
                pose.pose.pose.orientation = t.orientation
                pose.pose.covariance = [
                    0.05, 0, 0, 0, 0, 0,
                    0, 0.05, 0, 0, 0, 0,
                    0, 0, 0.01, 0, 0, 0,
                    0, 0, 0, 0.01, 0, 0,
                    0, 0, 0, 0, 0.01, 0,
                    0, 0, 0, 0, 0, 0.1
                ]
                self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
