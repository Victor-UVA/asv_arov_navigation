#!/usr/bin/env python3

import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionServer
from asv_arov_interfaces.action import NavigationAction
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseWithCovarianceStamped
from asv_arov_navigation.utils import build_pose_stamped, euler_from_quaternion

class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self.action_server = ActionServer(self, NavigationAction, 'navigation_action', self.navigation_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('use_sim', False)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value

        if self.use_sim :
            self.asv_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/asv/amcl_pose', self.asv_pose_callback, 1)
            self.arov_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/arov/amcl_pose', self.arov_pose_callback, 1)

        self.asv_pose = None
        self.arov_pose = None

        self.arov_nav = BasicNavigator(node_name="navigation_basic_navigator", namespace="arov")
        self.asv_nav = BasicNavigator(namespace="asv")
        self.leader_task = None
        self.follower_task = None

        self._loop_rate = self.create_rate(1.0, self.get_clock())

    def asv_pose_callback(self, data) :
        self.asv_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion(data.pose.pose.orientation)[2]]

    def arov_pose_callback(self, data) :
        self.arov_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion(data.pose.pose.orientation)[2]]

    def navigation_callback(self, msg):
        if msg.request.mode == 0:   # stop all tasks
            self.arov_nav.cancelTask()
            self.asv_nav.cancelTask()

        elif (msg.request.mode == 1) or (msg.request.mode == 2):
            if msg.request.mode == 1:   # ASV leads, AROV follows
                leader_nav = self.asv_nav
                follower_nav = self.arov_nav
                leader_initial_pose = self._get_initial_pose('asv')
                follower_initial_pose = self._get_initial_pose('arov')
            else:   # AROV leads, ASV follows
                leader_nav = self.arov_nav
                follower_nav = self.asv_nav
                leader_initial_pose = self._get_initial_pose('arov')
                follower_initial_pose = self._get_initial_pose('asv')

            self.get_logger().info("Defined initial poses")

            leader_current_pose = None
            follower_current_pose = None
            follower_running = False
            leader_target_pose = build_pose_stamped(self.get_clock().now(), "map", [msg.request.goal[0], msg.request.goal[1], 0, 0, 0, msg.request.goal[2]])

            self.get_logger().info("Defined target pose")

            leader_nav.setInitialPose(leader_initial_pose)
            follower_nav.setInitialPose(follower_initial_pose)

            self.get_logger().info("Set initial poses")
            if msg.request.mode == 1 :
                leader_nav.waitUntilNav2Active(localizer="/asv/pose_publisher")
                follower_nav.waitUntilNav2Active(localizer="/arov/pose_publisher")
            else :
                leader_nav.waitUntilNav2Active(localizer="/arov/pose_publisher")
                follower_nav.waitUntilNav2Active(localizer="/asv/pose_publisher")

            self.get_logger().info("Nav2 active")

            self.leader_task = leader_nav.goToPose(leader_target_pose)

            self.get_logger().info("Completed goToPose call")

            while not leader_nav.isTaskComplete(): # or follower_running:
                leader_current_pose = leader_nav.getFeedback().current_pose
                if not follower_running:
                    follower_current_pose = follower_initial_pose
                else:
                    follower_current_pose = follower_nav.getFeedback().current_pose
                follower_target_pose = self._calculate_pose(follower_current_pose, leader_current_pose, 1)
                follower_nav.cancelTask()
                self.follower_task = follower_nav.goToPose(follower_target_pose)
                self._loop_rate.sleep()
                if not follower_nav.isTaskComplete():
                    follower_running = True
                else:
                    follower_running = False

        elif(msg.request.mode == 3) or (msg.request.mode == 4):
            if (msg.request.mode == 3): # only ASV moves
                leader_nav = self.asv_nav
                leader_initial_pose = self._get_initial_pose('asv')
            else:   # only AROV moves
                leader_nav = self.arov_nav
                leader_initial_pose = self._get_initial_pose('arov')

            leader_target_pose = build_pose_stamped(self.get_clock().now(), "map", [msg.request.goal[0], msg.request.goal[1], 0, 0, 0, msg.request.goal[2]])
            self.get_logger().info("Defined target pose")
            self.get_logger().info(f'{leader_initial_pose}')
            leader_nav.setInitialPose(leader_initial_pose)
            self.get_logger().info("Set initial poses")

            if msg.request.mode == 3:
                leader_nav.waitUntilNav2Active(localizer="/asv/pose_publisher")
            else:
                leader_nav.waitUntilNav2Active(localizer="/arov/pose_publisher")
            self.get_logger().info("Nav2 active")

            self.leader_task = leader_nav.goToPose(leader_target_pose)
            self.get_logger().info("Completed goToPose call")

        self.get_logger().info("Finishing navigation task")

        asv_result = self.asv_nav.getResult()
        # arov_result = self.arov_nav.getResult()
        result = NavigationAction.Result()
        if asv_result == TaskResult.SUCCEEDED:
            result.goal_reached = True
            return result
        else:
            result.goal_reached = False
            return result


    def _get_initial_pose(self, vehicle):
        initial_pose = PoseStamped()
        if not self.use_sim :
            while True :
                try:
                    transform = self.tf_buffer.lookup_transform(vehicle + '/base_link', 'map', rclpy.time.Time())
                    return build_pose_stamped(self.get_clock().now(), "map", [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z], transform.transform.rotation)
                except TransformException as initial_pose_ex:
                    self.get_logger().warning(f'Could not get {vehicle} initial pose: {initial_pose_ex}')
        else :
            while True :
                if (vehicle == 'asv' and self.asv_pose is not None) or (vehicle == 'arov' and self.arov_pose is not None) :
                    if vehicle == 'asv' :
                        return build_pose_stamped(self.get_clock().now(), "map", [self.asv_pose[0], self.asv_pose[1], 0, 0, 0, self.asv_pose[2]])
                    else :
                        return build_pose_stamped(self.get_clock().now(), "map", [self.arov_pose[0], self.arov_pose[1], 0, 0, 0, self.arov_pose[2]])

    def _calculate_pose(self, follower_pose, leader_pose, follower_clearance):
        x_transform = follower_pose.pose.position.x - leader_pose.pose.position.x
        y_transform = follower_pose.pose.position.y - leader_pose.pose.position.y
        xy_transform = np.array([x_transform, y_transform], dtype=np.float64)
        magnitude = np.linalg.norm(xy_transform)
        xy_transform_normalized = np.zeros(2, dtype=np.float64)
        if magnitude != 0 :
            xy_transform_normalized = xy_transform / np.linalg.norm(xy_transform)
        follow_orientation = Rotation.from_quat([follower_pose.pose.orientation.x, follower_pose.pose.orientation.y, follower_pose.pose.orientation.z, follower_pose.pose.orientation.w]).as_euler("xyz", degrees=False)
        target_yaw = math.atan2(-y_transform, -x_transform)

        return build_pose_stamped(self.get_clock().now(), "map", [leader_pose.pose.position.x + xy_transform_normalized[0] * follower_clearance, leader_pose.pose.position.y + xy_transform_normalized[1] * follower_clearance, follower_pose.pose.position.z, follow_orientation[0], follow_orientation[1], target_yaw])

def main() -> None:
    rclpy.init()
    nav_server = NavigationActionServer()
    rclpy.spin(nav_server)

if __name__ == '__main__':
    main()
