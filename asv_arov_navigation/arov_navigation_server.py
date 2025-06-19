import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_simple_commander.robot_navigator import BasicNavigator
from asv_arov_interfaces.action import NavigationAction

class AROVNavigationServer(Node) :
    def __init__(self) :
        super().__init__('AROV_navigation_server')
        self.action_server = ActionServer(self, NavigationAction, 'AROV_navigation_action', self.execute_callback_async)
        self.navigator = BasicNavigator(namespace='arov')
        self.navigator.waitUntilNav2Active(localizer="/arov/pose_publisher")

    def execute_callback_async(self, goal_handle) :
        if goal_handle.request.mode == 1 :
            target_pose = goal_handle.request.goal
            self.navigator.goToPose(target_pose)
            while not self.navigator.isTaskComplete() :
                feedback = NavigationAction.Feedback()
                feedback.current_pose = self.navigator.getFeedback().current_pose
                goal_handle.publish_feedback(feedback)
            result = NavigationAction.Result()
            result.goal_reached = True if self.navigator.getResult().error_code == 0 else False
            return result
        else :
            self.navigator.cancelTask()
            result = NavigationAction.Result()
            result.goal_reached = False
            return result
    
def main(args=None) :
    rclpy.init()
    action_server = AROVNavigationServer()
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()