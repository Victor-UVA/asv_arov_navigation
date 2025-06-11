import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from asv_arov_interfaces.action import ControlModeAction


class ControlActionClient(Node):

    def __init__(self):
        super().__init__('control_action_client')
        self._action_client = ActionClient(self, ControlModeAction, 'control_action')

    def send_goal(self, mode):
        goal_msg = ControlModeAction.Goal()
        goal_msg.mode = mode

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = ControlActionClient()

    future = action_client.send_goal(1)

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
