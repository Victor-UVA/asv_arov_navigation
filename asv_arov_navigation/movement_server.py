#!/usr/bin/env python3

import rclpy
import math
import numpy as np
import threading
import time

from rlcpy.duration import Duration
from rclpy.node import Node
from rclpy.action import ActionServer
from package.actions import NavigationAction

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool, Float32, Int32


class NavigationActionServer(Node):
    def __init__(self):
        super().__init__('Navigation_action_server')
        self.action_server = ActionServer(self, NavigationAction, 'Navigation_action_server', self.navigation_callback)
        self.arov_current_pose = None
        self.asv_current_pose = None
        self.l_task = None
        self.f_task = None
        self.leader_running = False
        self.leader_pose = None
        self.follower_pose = None
        self.leader_nav = None
        self.follower_nav = None
        self.follow_clearance = 1
        self.lead_task = None
        self.follow_task = None

    def navigation_callback(self, msg):
        if msg.mode == 0:   # stop all tasks, stand still
            self.leader_nav.cancelTask()
            self.follower_nav.cancelTask()

        elif (msg.mode == 1) or (msg.mode == 2):
            asv_transform = None
            arov_transform = None
            try:
                asv_transform = self.tf_buffer.lookup_transform('asv', 'map', rclpy.time.Time())
                self.asv_current_pose = PoseStamped()
                self.asv_current_pose.header = asv_transform.header
                self.asv_current_pose.pose.position.x = asv_transform.transform.translation.x
                self.asv_current_pose.pose.position.y = asv_transform.transform.translation.y
                self.asv_current_pose.pose.position.z = asv_transform.transform.translation.z
                self.asv_current_pose.pose.orientation = asv_transform.transform.rotation
            except TransformException as asv_ex:
                self.get_logger().info(f'Could not get ASV pose as transform: {asv_ex}')
            try:
                arov_transform = self.tf_buffer.lookup_transform('arov', 'map', rclpy.time.Time())
                self.arov_current_pose = PoseStamped()
                self.arov_current_pose.header = arov_transform.header
                self.arov_current_pose.pose.position.x = arov_transform.transform.translation.x
                self.arov_current_pose.pose.position.y = arov_transform.transform.translation.y
                self.arov_current_pose.pose.position.z = arov_transform.transform.translation.z
                self.arov_current_pose.pose.orientation = arov_transform.transform.rotation
            except TransformException as arov_ex:
                self.get_logger().info(f'Could not get AROV pose as transform: {arov_ex}')

            if msg.mode == 0:   # ASV leads
                thread1 = threading.Thread(target = self._move_to_target(0, self.asv_current_pose, msg.goal))
                thread2 = threading.Thread(target = self._follow_target(1, self.arov_current_pose))
                thread1.start()
                thread2.start()
                thread1.join()
                thread2.join()
            if msg.mode == 1:   # AROV leads
                thread1 = threading.Thread(target = self._move_to_target(1, self.arov_current_pose, msg.goal))
                thread2 = threading.Thread(target = self._follow_target(0, self.asv_current_pose))
                thread1.start()
                thread2.start()
                thread1.join()
                thread2.join()

    def _move_to_target(self, mode, initial_pose, target_pose):
        if mode == 0:   # asv lead
            self.leader_nav = BasicNavigator("asv")
            self.leader_nav.changeMap("/path/to/asv_costmap_params.yaml")
        else:   # arov lead
            self.leader_nav = BasicNavigator("arov")
            self.leader_nav.changeMap("/path/to/arov_costmap_params.yaml")

        self.leader_nav.setInitialPose(initial_pose)
        self.leader_nav.waitUntilNav2Active()
        self.lead_task = self.leader_nav.goToPose(target_pose)

        self.leader_running = True
        while not self.leader_nav.isTaskComplete(task = self.lead_task) and self.leader_running:
            self.leader_pose = self.leader_nav.getFeedback(task = self.lead_task).current_pose
        self.leader_running = False
        self.leader_nav.lifecycleShutdown()

    def _follow_target(self, mode, initial_pose):
        if mode == 0:   # arov follow asv
            self.follower_nav = BasicNavigator("arov")
            self.follower_nav.changeMap("/path/to/arov_costmap_params.yaml")
        else:   # asv follow arov
            self.follower_nav = BasicNavigator("asv")
            self.follower_nav.changeMap("/path/to/asv_costmap_params.yaml")

        self.follower_nav.setInitialPose(initial_pose)
        self.follower_nav.waitUntilNav2Active()
        while self.leader_running:
            target_pose = self._calculate_pose(self.follower_pose, self.leader_pose)
            self.follow_task = self.follower_nav.goToPose(target_pose)
            time.sleep(1)
        self.follower_nav.lifecycleShutdown()

    def _calculate_pose(self, follow_pose, lead_pose):
        x_transform = follow_pose.pose.position.x - lead_pose.pose.position.x
        y_transform = follow_pose.pose.position.y - lead_pose.pose.position.y
        xy_transform = np.array([x_transform, y_transform], dtype=np.float64)
        xy_transform_normalized = xy_transform / np.linalg.norm(xy_transform)
        (follow_roll, follow_pitch, _) = euler_from_quaternion(follow_pose.pose.orientation)
        target_yaw = math.atan2(-y_transform, -x_transform)
        target_orientation = quaternion_from_euler(follow_roll, follow_pitch, target_yaw)

        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = self.get_clock().now()
        target_pose.pose.position.x = lead_pose.pose.position.x + xy_transform_normalized[0] * self.follow_clearance
        target_pose.pose.position.y = lead_pose.pose.position.y + xy_transform_normalized[1] * self.follow_clearance
        target_pose.pose.position.z = follow_pose.pose.position.z
        target_pose.pose.orientation.x = target_orientation[0]
        target_pose.pose.orientation.y = target_orientation[1]
        target_pose.pose.orientation.z = target_orientation[2]
        target_pose.pose.orientation.w = target_orientation[3]

        return target_pose

def main() -> None:
    rlcpy.init()
    move_server = NavigationActionServer()
    rlcpy.spin(move_server)

if __name__ == '__main__':
    main()