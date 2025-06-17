import rclpy
import math
from enum import Enum
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from robot_guidance.apriltag_navigation_client import ApriltagNavigationClient

from asv_arov_interfaces.action import ControlModeAction
from asv_arov_interfaces.action import NavigationAction

from geometry_msgs.msg import PoseWithCovarianceStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from asv_arov_navigation.utils import build_pose_stamped, euler_from_quaternion
    
class NavigationActionClient(Node) :

    def __init__(self) :
        super().__init__('navigation_action_client')
        self.action_client = ActionClient(self, NavigationAction, 'navigation_action')
        self.done = False
        self.result = None

    # stop is vehicle 0, ASV is vehicle 1, AROV is vehicle 2
    def send_goal(self, goal, vehicle) :
        goal_msg = NavigationAction.Goal()
        goal_msg.goal = goal
        goal_msg.mode = vehicle

        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future) :
        self.get_logger().info("Goal response")
        goal_handle = future.result()
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future) :
        self.get_logger().info("Get result")
        self.done = True
        self.result = future.result().result
    
class ControlState(Enum) :
    STARTING = 0
    CLEANING = 1
    NAVIGATING = 2
    
class ControlActionServer(Node) :

    def __init__(self, navigation_client, cleaning_client) :
        super().__init__('control_action_server')
        self.cleaning_action_client = cleaning_client
        self.navigation_action_client = navigation_client
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

        if self.use_sim :
            self.asv_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, '/asv/amcl_pose', self.asv_pose_callback, 1)

        self.asv_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asv_target_poses = [[0, 0, 0], [0, -3, 0]]
        self.asv_target_pose_id = 0
        self.asv_home_pose = None

        self.state = ControlState.STARTING
        self.cleaning_future = None
        self.cleaning_check = False
        self.navigation_future = None
        self.navigation_check = False

        self.fence_frame_cleaning_routine_poses = [build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, self.cleaning_routine_apriltag_offset, 0, 0, 0, 0])]
        self.fence_frame_cleaning_routine_directions = []

        for i in range(0, math.ceil(self.cleaning_routine_width / self.cleaning_routine_strip_width)) :
            previous_pos = self.fence_frame_cleaning_routine_poses[-1].pose.position
            strip_depth = self.cleaning_routine_depth if previous_pos.z == 0 else 0
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, previous_pos.y, strip_depth, 0, 0, 0]))
            self.fence_frame_cleaning_routine_directions.append("vertical")
            self.fence_frame_cleaning_routine_poses.append(build_pose_stamped(self.get_clock().now(), "map", [self.cleaning_routine_apriltag_clearance, previous_pos.y + self.cleaning_routine_strip_width, strip_depth, 0, 0, 0]))
            self.fence_frame_cleaning_routine_directions.append("right")

    def asv_pose_callback(self, data) :
        self.asv_pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion(data.pose.pose.orientation)[2]]
        
    def execute_callback_async(self, goal_handle) :
        if goal_handle.request.mode == 1 :
            while True :
                if self.state == ControlState.STARTING :
                    if self.asv_home_pose is None :
                        if not self.use_sim :
                            t = None
                            try :
                                t = self.tf_buffer.lookup_transform('asv/base_link', 'map', rclpy.time.Time())
                            except TransformException as ex :
                                self.get_logger().info(f'Could not get ASV pose as transform: {ex}')
                            if t is not None :
                                self.asv_home_pose = [0, 0, 0]
                                self.asv_home_pose[0] = t.transform.translation.x
                                self.asv_home_pose[1] = t.transform.translation.y
                                self.asv_home_pose[2] = euler_from_quaternion(t.transform.rotation)[2]
                                self.state = ControlState.NAVIGATING
                        elif self.asv_pose is not None :
                            self.asv_home_pose = [0, 0, 0]
                            self.asv_home_pose[0] = self.asv_pose[0]
                            self.asv_home_pose[1] = self.asv_pose[1]
                            self.asv_home_pose[2] = self.asv_pose[2]
                            self.state = ControlState.NAVIGATING
                elif self.state == ControlState.CLEANING :
                    # if not self.cleaning_check :
                    #     self.cleaning_check = True
                    #     self.cleaning_action_client.send_goal(None)
                    # elif self.cleaning_future is not None and self.cleaning_future.goal_reached :
                    #     self.cleaning_check = False
                        self.state = ControlState.NAVIGATING
                elif self.state == ControlState.NAVIGATING :
                    if not self.navigation_check :
                        self.get_logger().info("prenav")
                        self.navigation_check = True
                        if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                            self.asv_target_pose_id = 0
                        self.navigation_action_client.send_goal(self.asv_target_poses[self.asv_target_pose_id], 1)
                    elif self.navigation_action_client.done :
                        self.get_logger().info("postnav")
                        self.navigation_action_client.done = False
                        self.navigation_check = False
                        self.asv_target_pose_id += 1
                        self.state = ControlState.CLEANING
                    # self.get_logger().info(f'{self.navigation_future.result().get_result_async().done()}')
        elif goal_handle.mode == 0 :
            future = self.navigation_action_client.send_goal(self.asv_home_pose, 1)
            rclpy.spin_until_future_complete(self.navigation_action_client, future)
            return True

def main(args=None) :
    rclpy.init()
    navigation_client = NavigationActionClient()
    cleaning_client = ApriltagNavigationClient()
    action_server = ControlActionServer(navigation_client, cleaning_client)
    executor = MultiThreadedExecutor()
    executor.add_node(navigation_client)
    executor.add_node(cleaning_client)
    executor.add_node(action_server)
    executor.spin()

if __name__ == '__main__' :
    main()

