import rclpy
import math
import numpy as np
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from asv_arov_interfaces.action import ControlModeAction, NavigationAction
from robot_guidance_interfaces.action import NavigateAprilTags
from asv_arov_interfaces.srv import SetPump

from geometry_msgs.msg import PoseStamped, PoseArray

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asv_arov_navigation.utils import build_pose_stamped, euler_from_quaternion, quaternion_from_euler, transform_pose_stamped

class ControlState(Enum) :
    STARTING = 0
    CLEANING = 1
    NAVIGATING = 2
    
class ControlActionServer(Node) :

    def __init__(self) :
        super().__init__('control_action_server')
        self.cleaning_action_client = ActionClient(self, NavigateAprilTags, 'navigate_apriltags')
        self.navigation_action_client = ActionClient(self, NavigationAction, 'navigation_action')
        self.action_server = ActionServer(self, ControlModeAction, 'control_action', 
                                          execute_callback=self.execute_callback_async,
                                          cancel_callback=self.cancel_callback)
        self.toggle_cleaners_client = self.create_client(SetPump, '/set_pump')

        self.declare_parameter('use_sim', False)
        self.declare_parameter('cleaning_routine_depth', 0.0)
        self.declare_parameter('cleaning_routine_width', 0.0)
        self.declare_parameter('cleaning_routine_strip_width', 0.0)
        self.declare_parameter('cleaning_routine_apriltag_x_offset', 0.0)
        self.declare_parameter('cleaning_routine_apriltag_y_offset', 0.0)
        self.declare_parameter('cleaning_routine_apriltag_clearance', 0.0)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.cleaning_routine_depth: float = self.get_parameter('cleaning_routine_depth').get_parameter_value().double_value
        self.cleaning_routine_width: float = self.get_parameter('cleaning_routine_width').get_parameter_value().double_value
        self.cleaning_routine_strip_width: float = self.get_parameter('cleaning_routine_strip_width').get_parameter_value().double_value
        self.cleaning_routine_apriltag_x_offset: float = self.get_parameter('cleaning_routine_apriltag_x_offset').get_parameter_value().double_value
        self.cleaning_routine_apriltag_y_offset: float = self.get_parameter('cleaning_routine_apriltag_y_offset').get_parameter_value().double_value
        self.cleaning_routine_apriltag_clearance: float = self.get_parameter('cleaning_routine_apriltag_clearance').get_parameter_value().double_value

        self.pose_pub = self.create_publisher(PoseArray, "pose_array", 10)

        self.asv_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asv_target_poses = [build_pose_stamped(self.get_clock().now(), "map", [0.5, 0, 0, 0, 0, 0]), build_pose_stamped(self.get_clock().now(), "map", [0, 3, 0, 0, 0, 0])]
        self.arov_fence_frame_pairs = [["tag36h11:1", "tag36h11:2"], ["tag36h11:3", "tag36h11:4"]]
        self.arov_fence_switch = 0
        self.asv_target_pose_id = 1
        self.asv_home_pose = None

        self.state = ControlState.STARTING
        self.cleaning_check = False
        self.navigation_check = False
        
        self.nav_done = False
        self.cleaning_done = False
        self.nav_goal_handle = None
        self.cleaning_goal_handle = None

        self.fence_frame_cleaning_routine_poses = [build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_x_offset, self.cleaning_routine_apriltag_y_offset, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0])]
        self.fence_frame_cleaning_routine_directions = []

        for i in range(0, math.ceil(self.cleaning_routine_width / self.cleaning_routine_strip_width)) :
            previous_pos = self.fence_frame_cleaning_routine_poses[-1].pose.position
            strip_depth = self.cleaning_routine_depth if previous_pos.y == 0 else 0
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [previous_pos.x, strip_depth, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0]))
            self.fence_frame_cleaning_routine_directions.append("vertical")
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [previous_pos.x + self.cleaning_routine_strip_width, strip_depth, self.cleaning_routine_apriltag_clearance, 0, math.pi/2, 0]))
            self.fence_frame_cleaning_routine_directions.append("left")

        self.timer = None
    
    def toggle_cleaners(self, active) :
        self.get_logger().info("Activating cleaner pump" if active else "Deactivating cleaner pump")
        if not self.use_sim :
            request = SetPump.Request()
            request.set_pump = active
            self.toggle_cleaners_client.wait_for_service()
            self.toggle_cleaners_client.call_async(request)
    
    def send_navigation_goal(self, goal, vehicle) :
        goal_msg = NavigationAction.Goal()
        goal_msg.goal = goal
        goal_msg.mode = vehicle
        self.nav_done = False

        self.navigation_action_client.wait_for_server()
        future = self.navigation_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future) :
        self.nav_goal_handle = future.result()
        result_future = future.result().get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future) :
        self.nav_done = True

    def send_cleaning_goal(self, pose_list, commands) :
        goal_msg = NavigateAprilTags.Goal()
        goal_msg.goals = pose_list
        goal_msg.commands = commands
        self.cleaning_done = False

        self.cleaning_action_client.wait_for_server()
        future = self.cleaning_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.cleaning_goal_response_callback)

    def cleaning_goal_response_callback(self, future) :
        self.cleaning_goal_handle = future.result()
        result_future = future.result().get_result_async()
        result_future.add_done_callback(self.cleaning_result_callback)

    def cleaning_result_callback(self, future) :
        self.cleaning_done = True
        
    def execute_callback_async(self, goal_handle) :
        if goal_handle.request.mode == 0 :
            if self.timer is not None :
                self.timer.destroy()
            if self.cleaning_done is not None :
                self.cleaning_goal_handle.cancel()
            if self.nav_goal_handle is not None :
                self.nav_goal_handle.cancel()
        elif goal_handle.request.mode == 1 :
            self.timer = self.create_timer(1, self.run_state_machine)
        elif goal_handle.request.mode == 2 :
            if self.timer is not None :
                self.timer.destroy()
            if self.cleaning_done is not None :
                self.cleaning_goal_handle.cancel()
            if self.nav_goal_handle is not None :
                self.nav_goal_handle.cancel()
            self.send_navigation_goal(self.asv_home_pose, 1)
        return ControlModeAction.Result()

    def publish_poses(self, pose_list) :
        msg = PoseArray()
        msg.header.frame_id = 'map'
        pose_only_list = [p.pose for p in pose_list]
        msg.poses = pose_only_list
        self.pose_pub.publish(msg)

    def setup_routine_in_frame(self, frame_id) :
        out = []
        t = None
        while True :
            try :
                t = self.tf_buffer.lookup_transform('map', frame_id, rclpy.time.Time())
                break
            except TransformException as ex :
                self.get_logger().info(f'Could not get AprilTag transform: {ex}')
        for i in self.fence_frame_cleaning_routine_poses :
            self.get_logger().info(f'Pose in AprilTag frame: {i.pose.position} \n Orientation in AprilTag frame: {i.pose.orientation} \n Translation from {frame_id}: {t.transform.translation} \n Rotation from {frame_id}: {t.transform.rotation}')
            out.append(transform_pose_stamped(i, t))
            self.get_logger().info(f'Pose in map frame: {out[-1].pose.position} \n Orientation in map frame: {out[-1].pose.orientation}')
        self.publish_poses(out)
        return out
    
    def run_state_machine(self) :
        if self.state == ControlState.STARTING :
            if self.asv_home_pose is None :
                t = None
                try :
                    t = self.tf_buffer.lookup_transform('map', 'asv/base_link', rclpy.time.Time())
                except TransformException as ex :
                    self.get_logger().info(f'Could not get ASV pose as transform: {ex}')
                if t is not None :
                    self.asv_home_pose = build_pose_stamped(self.get_clock().now(), "map", [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], t.transform.orientation)
            else :
                self.state = ControlState.NAVIGATING
        elif self.state == ControlState.CLEANING :
            if not self.cleaning_check :
                self.get_logger().info("Cleaning start")
                self.cleaning_check = True
                self.toggle_cleaners(True)
                self.send_cleaning_goal(self.setup_routine_in_frame(self.arov_fence_frame_pairs[self.asv_target_pose_id][self.arov_fence_switch]), self.fence_frame_cleaning_routine_directions)
            elif self.cleaning_done :
                self.get_logger().info("Cleaning end")
                self.toggle_cleaners(False)
                self.cleaning_check = False
                self.cleaning_action_client.done = False
                if self.arov_fence_switch == 1 :
                    self.arov_fence_switch = 0
                    self.state = ControlState.NAVIGATING
                else :
                    self.arov_fence_switch = 1
        elif self.state == ControlState.NAVIGATING :
            if not self.navigation_check :
                self.get_logger().info("Nav start")
                self.navigation_check = True
                self.asv_target_pose_id += 1
                if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                    self.asv_target_pose_id = 0
                self.send_navigation_goal(self.asv_target_poses[self.asv_target_pose_id], 1)
            elif self.nav_done :
                self.get_logger().info("Nav end")
                self.navigation_check = False
                self.state = ControlState.CLEANING

def main(args=None) :
    rclpy.init()
    action_server = ControlActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()
