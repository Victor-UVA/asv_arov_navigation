import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from asv_arov_interfaces.srv import DemoMode
from asv_arov_interfaces.action import ControlModeAction

class ControlClient(Node) :
    def __init__(self) :
        super().__init__('control_client')
        self.demo_service = self.create_service(DemoMode, 'demo_mode', self.demo_callback)
        self.action_client = ActionClient(self, ControlModeAction, 'control_action_server')

    def demo_callback(self, request, response) :
        goal_msg = ControlModeAction.Goal()
        goal_msg.mode = 0 if request.mode == "STOP" else 1 if request.mode == "START" else 2
        self.action_client.wait_for_server()
        self.action_client.send_goal_async(goal_msg)
        response.success = True
        return response