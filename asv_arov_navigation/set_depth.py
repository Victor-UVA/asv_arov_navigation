import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from robot_guidance_interfaces.action import GoToDepth

class Client(Node) :
    def __init__(self) :
        super().__init__('set_depth_test_client')
        self.action_client = ActionClient(self, GoToDepth, 'go_to_depth')
    def send_goal(self, depth) :
        goal_msg = GoToDepth.Goal()
        goal_msg.target_depth = depth
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)

def main(args=None) :
    rclpy.init()
    action_client = Client()
    future = action_client.send_goal(5.0)
    rclpy.spin_until_future_complete(action_client, future)

if __name__ == '__main__' :
    main()
