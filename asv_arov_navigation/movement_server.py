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

    def send_goal(self, goal, init_pose, mode):
        goal_msg = AROVCommandAction.Goal()
        goal_msg.goal = goal
        goal_msg.init_pose = init_pose
        goal_msg.mode = mode
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)

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
            leader_future = self.send_goal(leader_goal_pose, leader_initial_pose, 1)
        if follower_initial_pose is not None:
            follower_running = False
            while not self.asv_nav.isTaskComplete():
                leader_current_pose = self._get_initial_pose('asv')
                if not follower_running:
                    follower_current_pose = follower_initial_pose
                else:
                    follower_current_pose = self._get_initial_pose('arov')
                follower_goal_pose = self._calculate_pose(follower_current_pose, leader_current_pose, self.follow_distance)
                follower_future = self.send_goal(PoseStamped(), PoseStamped(), 0)
                follower_future = self.send_goal(follower_goal_pose, follower_current_pose, 1)
                if follower_future.done() and follower_future.result().get_result_async().done():
                    follower_running = False
                else:
                    follower_running = True
            if self.asv_nav.isTaskComplete():
                follower_goal_pose = self._get_initial_pose('asv')
                follower_current_pose = self._get_initial_pose('arov')
                follower_goal_pose.pose.position.z = follower_current_pose.pose.position.z
                follower_future = self.send_goal(PoseStamped(), PoseStamped(), 0)
                follower_future = self.send_goal(follower_goal_pose, follower_current_pose, 1)
                while not follower_future.done() or (follower_future.done() and not follower_future.result().get_result_async().done()):
                    print('Moving to waypoint')
            follower_success = follower_future.result().goal_reached

        else:
            if leader == 'arov':
                while not leader_future.done() or (leader_future.done() and not leader_future.get_result_async().done()):
                    print('Moving to waypoint')
                    pass
                leader_success = leader_future.result().goal_reached
            else:
                while not self.asv_nav.isTaskComplete():
                    print('Moving to waypoint')
                    pass
                leader_success = True if self.asv_nav.getResult().error_code == 0  else False
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
