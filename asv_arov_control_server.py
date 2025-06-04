import rclpy
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
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
    
class ControlState(Enum) :
    STARTING = 0
    CLEANING = 1
    NAVIGATING = 2
    
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

        self.state = ControlState.STARTING
        self.cleaner_future = None
        self.cleaner_check = False
        self.navigator_future = None
        self.navigator_check = False
        
    def execute_callback_async(self, goal_handle) :
        if goal_handle.mode == 1 :
            while True :
                if self.state == ControlState.STARTING :
                    if self.asv_home_pose is None :
                        t = None
                        try :
                            t = self.tf_buffer.lookup_transform('asv', 'map', rclpy.time.Time())
                        except TransformException as ex :
                            self.get_logger().info(f'Could not get ASV pose as transform: {ex}')
                        if t is not None :
                            self.asv_home_pose = PoseStamped()
                            self.asv_home_pose.header = t.header
                            self.asv_home_pose.pose.position.x = t.transform.translation.x
                            self.asv_home_pose.pose.position.y = t.transform.translation.y
                            self.asv_home_pose.pose.position.z = t.transform.translation.z
                            self.asv_home_pose.pose.orientation = t.transform.orientation
                            self.state = ControlState.NAVIGATING
                elif self.state == ControlState.CLEANING :
                    if not self.cleaner_check :
                        self.cleaner_check = True
                        self.cleaner_future = self.cleaner_action_client.send_goal(None)
                        rclpy.spin_until_future_complete(self.cleaner_action_client, self.cleaner_future)
                    elif self.cleaner_future is not None and self.cleaner_future.goal_reached :
                        self.cleaner_check = False
                        self.state = ControlState.NAVIGATING
                elif self.state == ControlState.NAVIGATING :
                    if not self.navigator_check :
                        self.navigator_check = True
                        if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                            self.asv_target_pose_id == 0
                        target = self.asv_target_poses[self.asv_target_pose_id]
                        target.header.stamp = rclpy.time.Time()
                        future = self.navigator_action_client.send_goal(target, 1)
                        rclpy.spin_until_future_complete(self.navigator_action_client, future)
                    elif self.navigator_future is not None and self.navigator_future.goal_reached :
                        self.navigator_check = False
                        self.asv_target_pose_id += 1
                        self.state = ControlState.CLEANING
        elif goal_handle.mode == 0 :
            self.asv_home_pose.header.stamp = rclpy.time.Time()
            future = self.navigator_action_client.send_goal(self.asv_home_pose, 1)
            rclpy.spin_until_future_complete(self.navigator_action_client, future)
            return True

def main(args=None) :
    rclpy.init(args)
    action_server = ControlActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()