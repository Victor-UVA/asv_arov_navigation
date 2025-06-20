#!/usr/bin/env python3

import rclpy
import math
import numpy as np

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from asv_arov_interfaces.action import NavigationAction, AROVCommandAction
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
        self.action_client = ActionClient(self, AROVCommandAction, 'AROV_navigation_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.declare_parameter('use_sim', False)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value

        self.asv_nav = BasicNavigator(namespace="asv")
        self.asv_nav.waitUntilNav2Active(localizer="/asv/pose_publisher")
        self.get_logger().info("ASV Nav2 Active")
        self.arov_nav = BasicNavigator(namespace="arov")
        self.arov_nav.waitUntilNav2Active(localizer="/arov/pose_publisher")
        self.get_logger().info("AROV Nav2 Active")

        self.follow_distance = 1.0
        self._sleep_rate = self.create_rate(2.0, self.get_clock())
        # self._loop_rate = self.create_rate(1.0, self.get_clock())

    def navigation_callback(self, msg):
        if msg.request.mode == 0:   # stop all tasks
            self.send_goal(PoseStamped(), PoseStamped(), 0) # AROV cancel task
            self.asv_nav.cancelTask()

        else:
            leader_goal_pose = msg.request.goal
            self.get_logger().info("Defined goal pose")

            if (msg.request.mode % 2) == 1:
                asv_initial_pose = self._get_initial_pose('asv')
                self.get_logger().info("Defined initial poses")
                leader_goal_pose.pose.position.z = asv_initial_pose.pose.position.z
                if msg.request.mode == 1:   # ASV leads, AROV follows
                    return self._goto('asv', asv_initial_pose, leader_goal_pose, True)
                else:   # only ASV moves
                    return self._goto('asv', asv_initial_pose, leader_goal_pose, False)
            else:
                arov_initial_pose = self._get_initial_pose('arov')
                self.get_logger().info("Defined initial poses")
                leader_goal_pose.pose.position.z = arov_initial_pose.pose.position.z
                if msg.request.mode == 2:   # AROV leads, ASV follows
                    return self._goto('arov', arov_initial_pose, leader_goal_pose, True)
                else:   # only AROV moves
                    return self._goto('arov', arov_initial_pose, leader_goal_pose, False)

    def _goto(self, leader, leader_initial_pose, leader_goal_pose, follower_enabled):
        leader_success = False
        follower_success = True
        if leader == 'asv' and follower_enabled:
            lead_nav = self.asv_nav
            self.get_logger().info("ASV set as leader")
        else:
            lead_nav = self.arov_nav
            self.get_logger().info("AROV set as leader")

        lead_nav.setInitialPose(leader_initial_pose)
        self.get_logger().info(f"Set {leader}'s initial pose")
        lead_nav.goToPose(leader_goal_pose)
        self.get_logger().info(f"Completed {leader}'s goToPose call")

        if follower_enabled:
            if leader == 'asv':
                follower = 'arov'
                follow_nav = self.arov_nav
                self.get_logger().info("AROV set as follower")
            else:
                follower = 'asv'
                follow_nav = self.asv_nav
                self.get_logger().info("ASV set as follower")

            self._sleep_rate.sleep()  # let leader move away
            while not lead_nav.isTaskComplete():
                leader_current_pose = self._get_initial_pose(leader)
                follower_current_pose = self._get_initial_pose(follower)
                follower_goal_pose = self._calculate_pose(follower_current_pose, leader_current_pose, self.follow_distance)

                # self._sleep_rate.sleep()    # let leader move away on first loop, let follower move on subsequent loops

                follow_nav.setInitialPose(follower_current_pose)
                self.get_logger().info(f"Got {follower}'s initial pose")
                follow_nav.cancelTask()
                self.get_logger().info(f"Canceled {follower}'s current goto")
                follow_nav.goToPose(follower_goal_pose)
                self.get_logger().info(f"Completed {follower}'s goToPose call")

            if lead_nav.isTaskComplete():
                leader_success = True if lead_nav.getResult().error_code == 0 else False
                self.get_logger().info(f"Leader finished moving - success: {leader_success}")

                follower_goal_pose = self._get_initial_pose(leader)
                follower_current_pose = self._get_initial_pose(follower)
                follower_goal_pose.pose.position.z = follower_current_pose.pose.position.z

                follow_nav.setInitialPose(follower_current_pose)
                #self.get_logger().info(f"Got {follower}'s initial pose")
                follow_nav.cancelTask()
                #self.get_logger().info(f"Canceled {follower}'s current goto")
                follow_nav.goToPose(follower_goal_pose)
                #self.get_logger().info(f"Completed {follower}'s goToPose call")

                while not follow_nav.isTaskComplete():
                    pass
                follower_success = True if self.asv_nav.getResult.error_code == 0 else False
                self.get_logger().info(f"Follower finished moving - success: {follower_success}")

        else:
            while not lead_nav.isTaskComplete():
                pass
            leader_success = True if lead_nav.getResult() == 0 else False
            self.get_logger().info(f"Leader finished moving - success: {leader_success}")

        self.get_logger().info(f"Done with navigation action(s) - success: {leader_success and follower_success}")
        result = NavigationAction.Result()
        result.goal_reached = leader_success and follower_success
        return result

    def _get_initial_pose(self, vehicle):
        while True :
            try:
                transform = self.tf_buffer.lookup_transform(vehicle + '/base_link', 'map', rclpy.time.Time())
                return build_pose_stamped(self.get_clock().now(), "map", [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z], transform.transform.rotation)
            except TransformException as initial_pose_ex:
                self.get_logger().warning(f'Could not get {vehicle} initial pose: {initial_pose_ex}')

    def _calculate_pose(self, follower_pose, leader_pose, follower_clearance):
        dx = leader_pose.pose.position.x - follower_pose.pose.position.x
        dy = leader_pose.pose.position.y - follower_pose.pose.position.y
        psi = math.atan2(dy, dx)
        goal_x = leader_pose.pose.position.x - math.cos(psi) * follower_clearance
        goal_y = leader_pose.pose.position.y - math.sin(psi) * follower_clearance

        return build_pose_stamped(self.get_clock().now(), "map", [goal_x, goal_y, follower_pose.pose.position.z, 0, 0, psi - math.pi])
    
def main() -> None:
    rclpy.init()
    nav_server = NavigationActionServer()
    rclpy.spin(nav_server)

if __name__ == '__main__':
    main()
