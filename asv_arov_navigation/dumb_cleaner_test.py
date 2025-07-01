import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from asv_arov_interfaces.action import DumbCleanAction

class TestCleanerClient(Node) :
    def __init__(self) :
        super().__init__('dumb_cleaner_test_client')
        self.client = ActionClient(self, DumbCleanAction, 'dumb_cleaner')

    def send_goal(self, bars, switch_start_state) :
        goal_msg = DumbCleanAction.Goal()
        goal_msg.bars = bars
        goal_msg.switch_start_state = switch_start_state
        self.client.wait_for_server()
        return self.client.send_goal_async(goal_msg)

def main(args=None) :
    rclpy.init()
    node = TestCleanerClient()
    rclpy.spin_until_future_complete(node, node.send_goal(4, -1))

if __name__ == '__main__' :
    main()