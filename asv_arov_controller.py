# By James Farnsworth

import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from package.actions import ControlModeAction

from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler

class CleanerActionClient(Node) :

    def __init__(self) :
        super().__init__('cleaner_action_client')
        self.action_client = ActionClient(self, PlaceholderAction, 'cleaner_action_server')

    def send_goal(self, request) :
        goal_msg = PlaceholderAction.Goal()
        goal_msg.request = request

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)
        

class ControllerActionServer(Node) :

    def __init__(self) :
        super().__init__('controller_action_server')
        self.arov_pose_subscriber = self.create_subscription(PoseStamped, 'topic', self.arov_pose_callback, 10)
        self.asv_pose_subscriber = self.create_subsrciption(PoseStamped, 'topic', self.asv_pose_callback, 10)
        self.arov_target_pose_publisher = self.create_publisher(PoseStamped, 'arov_target_pose', 1)
        self.asv_target_pose_publisher = self.create_publisher(PoseStamped, 'asv_target_pose', 1)
        self.action_server = ActionServer(self, ControlModeAction, 'asv_arov_controller', self.execute_callback)
        # self.action_client = CleanerActionClient()

        self.arov_pose = None
        self.arov_follow_clearance = 1 # minimum distance from ASV in meters
        self.asv_pose = None
        self.asv_target_poses = []
        self.asv_target_pose_id = 0
        self.asv_target_clearance = 1 # minimum distance from target to be considered to have arrived in meters

        self.home = None
        self.has_set_home = False

    def arov_pose_callback(self, pose) :
        self.arov_pose = pose.pose

    def asv_pose_callback(self, pose) :
        self.asv_pose = pose.pose
    
    def execute_callback(self, goal_handle) :
        if not self.has_set_home :
            self.home = self.asv_pose
            self.has_set_home = True
        while True :

            # AROV follows ASV
            x_transform = self.arov_pose.pose.position.x - self.asv_pose.pose.position.x
            y_transform = self.arov_pose.pose.position.y - self.asv_pose.pose.position.y
            xy_transform = np.array([x_transform, y_transform], dtype=np.float64)
            xy_transform_normalized = xy_transform / np.linalg.norm(xy_transform)
                
            arov_orientation = self.arov_pose.pose.orientation
            (arov_roll, arov_pitch, arov_yaw) = euler_from_quaternion([arov_orientation.w, arov_orientation.x, arov_orientation.y, arov_orientation.z])
            # Set roll here if desired
            # Set pitch here if desired
            arov_yaw = math.atan2(-y_transform, -x_transform)
            arov_quat_orientation = quaternion_from_euler(arov_roll, arov_pitch, arov_yaw)

            arov_target_pose = PoseStamped()
            arov_target_pose.pose.position.x = self.asv_pose.pose.position.x + xy_transform_normalized[0] * self.arov_follow_clearance
            arov_target_pose.pose.position.y = self.asv_pose.pose.position.y + xy_transform_normalized[1] * self.arov_follow_clearance
            arov_target_pose.pose.position.z = self.arov_pose.pose.position.z # Add z control?
            arov_target_pose.pose.orientation.x = arov_quat_orientation[0]
            arov_target_pose.pose.orientation.y = arov_quat_orientation[1]
            arov_target_pose.pose.orientation.z = arov_quat_orientation[2]
            arov_target_pose.pose.orientation.w = arov_quat_orientation[3]
            arov_target_pose.header.frame_id = 'map'
            arov_target_pose.header.stamp = self.get_clock().now()

            self.arov_target_pose_publisher.publish(arov_target_pose)

            if goal_handle.request.mode == 1 :
                # ASV moves to goal
                x_error = self.asv_target_poses[self.asv_target_pose_id].position.x - self.asv_pose.pose.position.x
                y_error = self.asv_target_poses[self.asv_target_pose_id].position.y - self.asv_pose.pose.position.y
                r_error = np.linalg.norm(np.array([x_error, y_error], dtype=np.float64))
                if r_error < self.asv_target_clearance :
                    # future = self.action_client.send_goal()
                    # rclpy.spin_until_future_complete(self.action_client, future)
                    self.asv_target_pose_id = (self.asv_target_pose_id + 1) % len(self.asv_target_poses)

                asv_target_pose = PoseStamped()
                asv_target_pose.pose = self.asv_target_poses[self.asv_target_pose_id]
                asv_target_pose.header.frame_id = 'map'
                asv_target_pose.header.stamp = self.get_clock().now()

                # Publish target poses
                self.asv_target_pose_publisher.publish(asv_target_pose)
            elif goal_handle.request.mode == 0 :
                # ASV moves to home
                self.asv_target_pose_publisher.publish(self.home)

def main(args=None) :
    rclpy.init(args=args)
    controller_action_server = ControllerActionServer()
    rclpy.spin(controller_action_server)

if __name__ == 'main' :
    main()