import rclpy
import math
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from asv_arov_interfaces.action import ControlModeAction
from asv_arov_interfaces.action import NavigationAction
from robot_guidance_interfaces.action import NavigateAprilTags

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asv_arov_navigation.utils import build_pose_stamped, euler_from_quaternion

class ControlState(Enum) :
    STARTING = 0
    CLEANING = 1
    NAVIGATING = 2
    
class ControlActionServer(Node) :

    def __init__(self) :
        super().__init__('control_action_server')
        self.cleaning_action_client = ActionClient(self, NavigateAprilTags, 'navigate_apriltags')
        self.navigation_action_client = ActionClient(self, NavigationAction, 'navigation_action')
        self.action_server = ActionServer(self, ControlModeAction, 'control_action', self.execute_callback_async)

        self.declare_parameter('use_sim', False)
        self.declare_parameter('cleaning_routine_depth', 0.0)
        self.declare_parameter('cleaning_routine_width', 0.0)
        self.declare_parameter('cleaning_routine_strip_width', 0.0)
        self.declare_parameter('cleaning_routine_apriltag_offset', 0.0)
        self.declare_parameter('cleaning_routine_apriltag_clearance', 0.0)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value
        self.cleaning_routine_depth: float = self.get_parameter('cleaning_routine_depth').get_parameter_value().double_value
        self.cleaning_routine_width: float = self.get_parameter('cleaning_routine_width').get_parameter_value().double_value
        self.cleaning_routine_strip_width: float = self.get_parameter('cleaning_routine_strip_width').get_parameter_value().double_value
        self.cleaning_routine_apriltag_offset: float = self.get_parameter('cleaning_routine_apriltag_offset').get_parameter_value().double_value
        self.cleaning_routine_apriltag_clearance: float = self.get_parameter('cleaning_routine_apriltag_clearance').get_parameter_value().double_value

        self.asv_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asv_target_poses = [build_pose_stamped(self.get_clock().now(), "map", [0, -3, 0, 0, 0, 0]), build_pose_stamped(self.get_clock().now(), "map", [0, 0, 0, 0, 0, 0])]
        self.arov_fence_frame_pairs = [("", ""), ("", "")]
        self.asv_target_pose_id = 0
        self.asv_home_pose = None

        self.state = ControlState.STARTING
        self.cleaning_check = False
        self.navigation_check = False
        
        self.nav_done = False
        self.cleaning_done = False

        self.fence_frame_cleaning_routine_poses = [build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, self.cleaning_routine_apriltag_offset, 0, 0, 0, 0])]
        self.fence_frame_cleaning_routine_directions = []

        for i in range(0, math.ceil(self.cleaning_routine_width / self.cleaning_routine_strip_width)) :
            previous_pos = self.fence_frame_cleaning_routine_poses[-1].pose.position
            strip_depth = self.cleaning_routine_depth if previous_pos.z == 0 else 0
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, previous_pos.y, strip_depth, 0, 0, 0]))
            self.fence_frame_cleaning_routine_directions.append("vertical")
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, previous_pos.y + self.cleaning_routine_strip_width, strip_depth, 0, 0, 0]))
            self.fence_frame_cleaning_routine_directions.append("right")

    def send_navigation_goal(self, goal, vehicle) :
        goal_msg = NavigationAction.Goal()
        goal_msg.goal = goal
        goal_msg.mode = vehicle
        self.nav_done = False

        self.navigation_action_client.wait_for_server()
        future = self.navigation_action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future) :
        result_future = future.result().get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future) :
        self.nav_done = True
        
    def execute_callback_async(self, goal_handle) :
        if goal_handle.request.mode == 1 :
            self.create_timer(1, self.run_state_machine)
        return ControlModeAction.Result()
    
    def setup_routine_in_frame(self, frame_id) :
        out = []
        for i in self.fence_frame_cleaning_routine_poses :
            out.append(build_pose_stamped(self.get_clock().now(), frame_id, [i.pose.position.x, i.pose.position.y, i.pose.position.z], i.pose.orientation))
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
                    self.asv_home_pose = [t.transform.translation.x, t.transform.translation.y, euler_from_quaternion(t.transform.rotation)[2]]
            else :
                self.state = ControlState.NAVIGATING
        elif self.state == ControlState.CLEANING :
            #if not self.cleaning_check :
            #    self.get_logger().info("Cleaning start")
            #    self.cleaning_check = True
            #    self.cleaning_future = self.send_cleaning_goal(self.fence_frame_cleaning_routine_poses, self.fence_frame_cleaning_routine_directions)
            #elif self.cleaning_done :
            #    self.get_logger().info("Cleaning end")
            #    self.cleaning_check = False
            #    self.cleaning_action_client.done = False
                self.state = ControlState.NAVIGATING
        elif self.state == ControlState.NAVIGATING :
            if not self.navigation_check :
                self.get_logger().info("Nav start")
                self.navigation_check = True
                if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                    self.asv_target_pose_id = 0
                self.send_navigation_goal(self.asv_target_poses[self.asv_target_pose_id], 1)
            elif self.nav_done :
                self.get_logger().info("Nav end")
                self.navigation_check = False
                self.asv_target_pose_id += 1
                self.state = ControlState.CLEANING

def main(args=None) :
    rclpy.init()
    action_server = ControlActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()
