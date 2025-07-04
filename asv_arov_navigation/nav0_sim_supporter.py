import rclpy
import math
from rclpy.node import Node
from tf2_ros import TransformException, TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose, Twist, PoseWithCovarianceStamped
from asv_arov_navigation.utils import quaternion_from_euler, euler_from_quaternion, build_transform_stamped

class Nav0SimSupporter(Node) :
    def __init__(self) :
        super().__init__('nav0_sim_supporter')

        self.use_sim = True
        self.surface_z = 0
        self.dt = 0.1

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        if self.use_sim :
            self.asv_cmd_vel_subscriber = self.create_subscription(Twist, 'asv/cmd_vel', self.asv_cmd_vel_callback, 1)
            self.arov_cmd_vel_subscriber = self.create_subscription(Twist, 'arov/cmd_vel', self.arov_cmd_vel_callback, 1)
            self.asv_pose = Pose()
            self.arov_pose = Pose()

        self.asv_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'asv/amcl_pose', 10)
        self.arov_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'arov/amcl_pose', 10)

        self.fence1 = build_transform_stamped(self.get_clock().now(), "map", "fence1", [0, 0, 0, 0, math.pi/2, -math.pi/2])

        self.create_timer(self.dt, self.timer_callback)

    def asv_cmd_vel_callback(self, data) :
        rpy = euler_from_quaternion(self.asv_pose.orientation)
        self.asv_pose.position.x += data.linear.x * math.cos(rpy[2]) * self.dt
        self.asv_pose.position.y += data.linear.x * math.sin(rpy[2]) * self.dt
        self.asv_pose.position.z = float(self.surface_z)
        self.asv_pose.orientation = quaternion_from_euler([rpy[0], rpy[1], rpy[2] + data.angular.z * self.dt])

    def arov_cmd_vel_callback(self, data) :
        rpy = euler_from_quaternion(self.arov_pose.orientation)
        self.arov_pose.position.x += (data.linear.x * math.cos(rpy[2]) - data.linear.y * math.sin(rpy[2])) * self.dt
        self.arov_pose.position.y += (data.linear.x * math.sin(rpy[2]) + data.linear.y * math.cos(rpy[2])) * self.dt
        self.arov_pose.position.z += data.linear.z * self.dt
        self.arov_pose.orientation = quaternion_from_euler([rpy[0], rpy[1], rpy[2] + data.angular.z * self.dt])

    def timer_callback(self) :
        if self.use_sim :
            asv_tf = build_transform_stamped(self.get_clock().now(), "map", "asv/base_link", [self.asv_pose.position.x,
                                                                                              self.asv_pose.position.y,
                                                                                              self.asv_pose.position.z],
                                                                                              self.asv_pose.orientation)
            arov_tf = build_transform_stamped(self.get_clock().now(), "map", "arov/base_link", [self.arov_pose.position.x,
                                                                                                self.arov_pose.position.y,
                                                                                                self.arov_pose.position.z],
                                                                                                self.arov_pose.orientation)

            self.tf_broadcaster.sendTransform(asv_tf)
            self.tf_broadcaster.sendTransform(arov_tf)
            self.tf_broadcaster.sendTransform(self.fence1)

            asv_pwcs = PoseWithCovarianceStamped()
            asv_pwcs.header.stamp = self.get_clock().now().to_msg()
            asv_pwcs.header.frame_id = "map"
            asv_pwcs.pose.pose = self.asv_pose
            arov_pwcs = PoseWithCovarianceStamped()
            arov_pwcs.header.stamp = self.get_clock().now().to_msg()
            arov_pwcs.header.frame_id = "map"
            arov_pwcs.pose.pose = self.arov_pose

            self.asv_pose_publisher.publish(asv_pwcs)
            self.arov_pose_publisher.publish(arov_pwcs)

def main(args=None) :
    rclpy.init()
    node = Nav0SimSupporter()
    rclpy.spin(node)

if __name__ == '__main__' :
    main()
