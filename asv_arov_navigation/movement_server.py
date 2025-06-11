#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import time

from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionServer
from asv_arov_interfaces.action import NavigationAction
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float32, Int32


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('navigation_action_server')
        self.action_server = ActionServer(self, NavigationAction, 'navigation_action', self.navigation_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arov_nav = BasicNavigator("arov")
        self.asv_nav = BasicNavigator("asv")
        self.leader_task = None
        self.follower_task = None

    def navigation_callback(self, msg):
        if msg.mode == 0:   # stop all tasks
            self.arov_nav.cancelTask()
            self.asv_nav.cancelTask()

        elif (msg.mode == 1) or (msg.mode == 2):
            if msg.mode == 1:   # ASV leads, AROV follows
                leader_nav = self.asv_nav
                follower_nav = self.arov_nav
                leader_initial_pose = self._get_initial_pose('asv')
                follower_initial_pose = self._get_initial_pose('arov')
            else:   # AROV leads, ASV follows
                leader_nav = self.arov_nav
                follower_nav = self.asv_nav
                leader_initial_pose = self._get_initial_pose('arov')
                follower_initial_pose = self._get_initial_pose('asv')

            leader_current_pose = None
            follower_current_pose = None
            follower_running = False
            leader_target_pose = PoseStamped()
            leader_target_pose.header.frame_id = "map"
            leader_target_pose.header.stamp = rclpy.time.Time()
            leader_target_pose.pose.position.x = msg.goal[0]
            leader_target_pose.pose.position.y = msg.goal[1]
            leader_target_pose.pose.position.z = leader_initial_pose.pose.position.z
            orientation = Rotation.from_euler("xyz", [0, 0, msg_goal[2]], degrees=False).as_quat()
            leader_target_pose.pose.orientation.x = orientation[0]
            leader_target_pose.pose.orientation.y = orientation[1]
            leader_target_pose.pose.orientation.z = orientation[2]
            leader_target_pose.pose.orientation.w = orientation[3]

            leader_nav.setInitialPose(leader_initial_pose)
            follower_nav.setInitialPose(follower_initial_pose)
            leader_nav.waitUntilNav2Active()
            follower_nav.waitUntilNav2Active()
            self.leader_task = leader_nav.goToPose(leader_target_pose)

            while not leader_nav.isTaskComplete(task = self.leader_task) or follower_running:
                leader_current_pose = leader_nav.getFeedback(task = self.leader_task).current_pose
                if not follower_running:
                    follower_current_pose = follower_initial_pose
                else:
                    follower_current_pose = follower_nav.getFeedback(task = self.leader_task).current_pose
                follower_target_pose = self._calculate_pose(follower_current_pose, leader_current_pose, 1)
                self.follower_task = self.follower_nav.goToPose(follower_target_pose)
                time.sleep(1)   # update follower at 1Hz
                if not follower_nav.isTaskComplete(task = self.follower_task):
                    follower_running = True
                else:
                    follower_running = False

    def _get_initial_pose(self, vehicle):
        initial_pose = PoseStamped()
        try:
            transform = self.tf_buffer.lookup_transform(vehicle, 'map', rclpy.time.Time())
            initial_pose.header.frame_id = transform.header
            initial_pose.pose.position.x = transform.transform.translation.x
            initial_pose.pose.position.y = transform.transform.translation.y
            initial_pose.pose.position.z = transform.transform.translation.z
            initial_pose.pose.orientation = transform.transform.rotation
            return initial_pose
        except TransformException as initial_pose_ex:
            node.get_logger().warning(f'Could not get {vehicle} initial pose: {initial_pose_ex}')

    def _calculate_pose(self, follower_pose, leader_pose, follower_clearance):
        x_transform = follower_pose.pose.position.x - leader_pose.pose.position.x
        y_transform = follower_pose.pose.position.y - leader_pose.pose.position.y
        xy_transform = np.array([x_transform, y_transform], dtype=np.float64)
        xy_transform_normalized = xy_transform / np.linalg.norm(xy_transform)
        follow_orientation = Rotation.from_quat(follower_pose.pose.orientation).as_euler("xyz", degrees=False)
        target_yaw = math.atan2(-y_transform, -x_transform)
        target_orientation = Rotation.from_euler("xyz", [target_orientation[0], target_orientation[1], target_yaw], degrees=False).as_quat()

        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rclpy.time.Time()
        target_pose.pose.position.x = leader_pose.pose.position.x + xy_transform_normalized[0] * follower_clearance
        target_pose.pose.position.y = leader_pose.pose.position.y + xy_transform_normalized[1] * follower_clearance
        target_pose.pose.position.z = follower_pose.pose.position.z
        target_pose.pose.orientation.x = target_orientation[0]
        target_pose.pose.orientation.y = target_orientation[1]
        target_pose.pose.orientation.z = target_orientation[2]
        target_pose.pose.orientation.w = target_orientation[3]

        return target_pose

def main() -> None:
    rclpy.init()
    nav_server = NavigationActionServer()
    rclpy.spin(nav_server)

if __name__ == '__main__':
    main()
