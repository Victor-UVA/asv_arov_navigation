import rclpy
from enum import Enum
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from asv_arov_interfaces.action import ControlModeAction
from asv_arov_interfaces.action import CleaningAction
from asv_arov_interfaces.action import NavigationAction

from turtlesim.msg import Pose

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scipy.spatial.transform import Rotation

class CleaningActionClient(Node) :

    def __init__(self) :
        super().__init__('cleaning_action_client')
        self.action_client = ActionClient(self, CleaningAction, 'cleaning_action')

    def send_goal(self, request) :
        goal_msg = CleaningAction.Goal()
        goal_msg.request = request

        self.action_client.wait_for_server()

        return self.action_client.send_goal_async(goal_msg)
    
class NavigationActionClient(Node) :

    def __init__(self) :
        super().__init__('navigation_action_client')
        self.action_client = ActionClient(self, NavigationAction, 'navigation_action')

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

    def __init__(self, navigation_client, cleaning_client) :
        super().__init__('control_action_server')
        # self.cleaning_action_client = cleaning_client
        self.navigation_action_client = navigation_client
        self.action_server = ActionServer(self, ControlModeAction, 'control_action', self.execute_callback_async)

        self.declare_parameter('use_sim', False)
        self.use_sim: bool = self.get_parameter('use_sim').get_parameter_value().bool_value

        if self.use_sim :
            self.asv_pose_subscriber = self.create_subscription(Pose, '/asv/robot_pose', self.asv_pose_callback, 1)

        self.asv_pose = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.asv_target_poses = [[0, 0, 0], [0, 3, 0]]
        self.asv_target_pose_id = 0
        self.asv_home_pose = None

        self.state = ControlState.STARTING
        self.cleaning_future = None
        self.cleaning_check = False
        self.navigation_future = None
        self.navigation_check = False

    def asv_pose_callback(self, data) :
        self.asv_pose = data
        
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
                                rpy = Rotation.from_quat([t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]).as_euler("xyz", degrees=False)
                                self.asv_home_pose[2] = rpy[2]
                                self.state = ControlState.NAVIGATING
                        elif self.asv_pose is not None :
                            self.asv_home_pose = [0, 0, 0]
                            self.asv_home_pose[0] = self.asv_pose.x
                            self.asv_home_pose[1] = self.asv_pose.y
                            self.asv_home_pose[2] = self.asv_pose.theta
                            self.state = ControlState.NAVIGATING
                elif self.state == ControlState.CLEANING :
                    # if not self.cleaning_check :
                    #     self.cleaning_check = True
                    #     self.cleaning_future = self.cleaning_action_client.send_goal(None)
                    #     rclpy.spin_until_future_complete(self.cleaning_action_client, self.cleaning_future)
                    # elif self.cleaning_future is not None and self.cleaning_future.goal_reached :
                    #     self.cleaning_check = False
                        self.state = ControlState.NAVIGATING
                elif self.state == ControlState.NAVIGATING :
                    if not self.navigation_check :
                        self.get_logger().info("prenav")
                        self.navigation_check = True
                        if self.asv_target_pose_id % len(self.asv_target_poses) == 0 :
                            self.asv_target_pose_id == 0
                        future = self.navigation_action_client.send_goal(self.asv_target_poses[self.asv_target_pose_id], 1)
                        rclpy.spin_until_future_complete(self.navigation_action_client, future)
                    elif self.navigation_future is not None and self.navigation_future.goal_reached :
                        self.get_logger().info("postnav")
                        self.navigation_check = False
                        self.asv_target_pose_id += 1
                        self.state = ControlState.CLEANING
        elif goal_handle.mode == 0 :
            future = self.navigation_action_client.send_goal(self.asv_home_pose, 1)
            rclpy.spin_until_future_complete(self.navigation_action_client, future)
            return True

def main(args=None) :
    rclpy.init()
    navigation_client = NavigationActionClient()
    cleaning_client = CleaningActionClient()
    action_server = ControlActionServer(navigation_client, cleaning_client)
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()

