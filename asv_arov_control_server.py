import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from package.actions import ControlModeAction
from package.actions import CleaningAction
from package.actions import NavigationAction

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class CleanerActionClient(Node) :

    def __init__(self) :
        super().__init__('cleaner_action_client')
        self.action_client = ActionClient(self, CleaningAction, 'cleaner_action_server')

    def send_goal(self, request) :
        goal_msg = CleaningAction.Goal()
        goal_msg.request = request

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)
    
class NavigatorActionClient(Node) :

    def __init__(self) :
        super().__init__('navigator_action_client')
        self.action_client = ActionClient(self, NavigationAction, 'navigator_action_server')

    # stop is vehicle 0, ASV is vehicle 1, AROV is vehicle 2
    def send_goal(self, goal, vehicle) :
        goal_msg = NavigationAction.Goal()
        goal_msg.goal = goal
        goal_msg.mode = vehicle

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)
    
class ControlActionServer(Node) :

    def __init__(self) :
        super().__init__('control_action_server')
        self.cleaner_action_client = CleanerActionClient()
        self.navigator_action_client = NavigatorActionClient()
        self.action_server = ActionServer(self, ControlModeAction, 'control_action_server', self.execute_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asv_target_poses = []
        self.asv_target_pose_id = 0
        self.asv_home_pose = None

    def execute_callback(self, goal_handle) :
        if self.asv_home_pose is None :
            try :
                t = self.tf_buffer.lookup_transform('asv', 'map', rclpy.time.Time())
                self.asv_home_pose = PoseStamped()
                self.asv_home_pose.header = t.header
                self.asv_home_pose.pose.position.x = t.transform.translation.x
                self.asv_home_pose.pose.position.y = t.transform.translation.y
                self.asv_home_pose.pose.position.z = t.transform.translation.z
                self.asv_home_pose.pose.orientation = t.transform.orientation
            except TransformException as ex :
                self.get_logger().info(f'Could not get ASV pose as transform: {ex}')
        if goal_handle.request.mode == 1 :
            while True :
                if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                    self.asv_target_pose_id == 0
                target = self.asv_target_poses[self.asv_target_pose_id]
                target.header.stamp = rclpy.time.Time()
                future = self.navigator_action_client.send_goal(target, 1)
                rclpy.spin_until_future_complete(self.navigator_action_client, future)
                if future.goal_reached :
                    self.asv_target_pose_id += 1
                    future = self.cleaner_action_client.send_goal(None)
                    rclpy.spin_until_future_complete(self.cleaner_action_client, future)
        elif goal_handle.request.mode == 0 and self.asv_home_pose is not None :
            self.asv_home_pose.header.stamp = rclpy.time.Time()
            future = self.navigator_action_client.send_goal(self.asv_home_pose, 1)
            rclpy.spin_until_future_complete(self.navigator_action_client, future)
            return future.goal_reached