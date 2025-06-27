import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from asv_arov_interfaces.srv import DemoMode
from asv_arov_interfaces.action import ControlModeAction

class ControlClient(Node) :
    def __init__(self) :
        super().__init__('control_client')
        self.demo_service = self.create_service(DemoMode, '/demo_mode', self.demo_callback)
        self.action_client = ActionClient(self, ControlModeAction, 'control_action')

    def demo_callback(self, request, response) :
        self.send_goal(request.mode)
        response.success = True
        return response

    def send_goal(self, mode) :
        goal_msg = ControlModeAction.Goal()
        goal_msg.mode = 2 if mode == "HOME" else 1 if mode == "START" else 0
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)

def main(args=None) :
    rclpy.init()
    control_client = ControlClient()
    #control_client.send_goal("START")
    rclpy.spin(control_client)

if __name__ == '__main__' :
    main()
