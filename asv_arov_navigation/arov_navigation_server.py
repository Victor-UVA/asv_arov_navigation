import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer
from nav2_simple_commander.robot_navigator import BasicNavigator
from asv_arov_interfaces.action import AROVCommandAction

class AROVNavigationServer(Node) :
    def __init__(self) :
        super().__init__('AROV_navigation_server')
        self.action_server = ActionServer(self, AROVCommandAction, 'AROV_navigation_action', self.execute_callback_async)
        self.navigator = BasicNavigator(namespace='arov')
        self.navigator.waitUntilNav2Active(localizer="/arov/pose_publisher")

    def execute_callback_async(self, goal_handle) :
        self.get_logger().info("Received goal")
        if goal_handle.request.mode == 1 :
            self.navigator.setInitialPose(goal_handle.request.init_pose)
            self.navigator.goToPose(goal_handle.request.goal)
            while not self.navigator.isTaskComplete() :
                feedback = AROVCommandAction.Feedback()
                feedback.current_pose = self.navigator.getFeedback().current_pose
                goal_handle.publish_feedback(feedback)
        else :
            self.navigator.cancelTask()
        result = AROVCommandAction.Result()
        result.goal_reached = True if goal_handle.request.mode == 0 or self.navigator.getResult() == 0 else False
        return result
    
def main(args=None) :
    rclpy.init()
    action_server = AROVNavigationServer()
    rclpy.spin(action_server)

if __name__ == '__main__' :
    main()
