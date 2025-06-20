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

        self.follow_distance = 1.0
        self.arov_done = False
        self.arov_success = False
        self._sleep_rate = self.create_rate(2.0, self.get_clock())

    def send_goal(self, goal, init_pose, mode):
        self.arov_done = False
        self.arov_success = False
        goal_msg = AROVCommandAction.Goal()
        goal_msg.goal = goal
        goal_msg.init_pose = init_pose
        goal_msg.mode = mode
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info("Goal response")
        goal_handle = future.result()
        future = goal_handle.get_result_async()
        future.add_done_callback(self.goal_request_callback)

    def goal_request_callback(self, future):
        self.get_logger().info("Goal done")
        self.arov_done = True
        self.arov_success = future.result().result.goal_reached
        self.get_logger().info(f"Request callback - done: {self.arov_done} / success: {self.arov_success}")

    def navigation_callback(self, msg):
        if msg.request.mode == 0:   # stop all tasks
            self.send_goal(PoseStamped(), PoseStamped(), 0) # AROV cancel task
            self.asv_nav.cancelTask()

        else:
            asv_current_pose = self._get_initial_pose('asv')
            arov_current_pose = self._get_initial_pose('arov')
            self.get_logger().info("Defined initial poses")
            leader_goal_pose = msg.request.goal
            self.get_logger().info("Defined goal pose")

            if msg.request.mode == 1 or msg.request.mode == 3:
                leader_goal_pose.pose.position.z = asv_current_pose.pose.position.z
                if msg.request.mode == 1:   # ASV leads, AROV follows
                    return self._goto('asv', asv_current_pose, arov_current_pose, leader_goal_pose)
                else:   # only ASV moves
                    return self._goto('asv', asv_current_pose, None, leader_goal_pose)
            else:
                leader_goal_pose.pose.position.z = arov_current_pose.pose.position.z
                if msg.request.mode == 2:   # AROV leads, ASV follows
                    return self._goto('arov', arov_current_pose, asv_current_pose, leader_goal_pose)
                else:   # only AROV moves
                    return self._goto('arov', arov_current_pose, None, leader_goal_pose)

    def _goto(self, leader, leader_initial_pose, follower_initial_pose, leader_goal_pose):
        leader_success = False
        follower_success = True
        if leader == 'asv':
            self.asv_nav.setInitialPose(leader_initial_pose)
            self.get_logger().info("Set ASV's initial poses")
            self.asv_nav.goToPose(leader_goal_pose)
            self.get_logger().info("Completed ASV's goToPose call")
        else:
            self.get_logger().info("send goal - arov leader")
            self.send_goal(leader_goal_pose, leader_initial_pose, 1)

        if follower_initial_pose is not None:
            follower_running = False
            self._sleep_rate.sleep()
            while not self.asv_nav.isTaskComplete():
                leader_current_pose = self._get_initial_pose('asv')
                if not follower_running:
                    follower_current_pose = follower_initial_pose
                else:
                    follower_current_pose = self._get_initial_pose('arov')
                follower_goal_pose = self._calculate_pose(follower_current_pose, leader_current_pose, self.follow_distance)
                self.get_logger().info("cancel goal - while loop")
                self.send_goal(PoseStamped(), PoseStamped(), 0)
                self.get_logger().info("send goal - while loop")
                self.send_goal(follower_goal_pose, follower_current_pose, 1)
                if not self.arov_done:
                    follower_running = False
                else:
                    follower_running = True
            if self.asv_nav.isTaskComplete():
                follower_goal_pose = self._get_initial_pose('asv')
                follower_current_pose = self._get_initial_pose('arov')
                follower_goal_pose.pose.position.z = follower_current_pose.pose.position.z
                self.get_logger().info("cancel goal - if statement")
                self.send_goal(PoseStamped(), PoseStamped(), 0)
                self.get_logger().info("send goal - if statement")
                self.send_goal(follower_goal_pose, follower_current_pose, 1)
                self.get_logger().info("Moving to waypoint - if statement")
                # while not self.arov_done:
                pass
                follower_success = self.arov_success
                self.get_logger().info(f"Follower finished moving - success: {follower_success}")

        else:
            if leader == 'arov':
                self.get_logger().info('Moving to waypoint')
                while not self.arov_done:
                    pass
                leader_success = self.arov_success
            else:
                self.get_logger().info('Moving to waypoint')
                while not self.asv_nav.isTaskComplete():
                    pass
                leader_success = True if self.asv_nav.getResult().error_code == 0  else False
            self.get_logger().info(f"Leader finished moving - success: {leader_success}")

        self.get_logger().info("Done")
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
