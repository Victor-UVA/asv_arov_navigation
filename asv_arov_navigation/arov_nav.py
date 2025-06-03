import rlcpy
from rlcpy.duration import Duration
from rlcpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, Float32


class ArovTargetNav(Node):
    def __init__(self):
        super().__init__('arov_target_pose')
        self.arov_target_sub = self.create_subscription(PoseStamped, 'arov_target_pose', self.target_callback, 1)
        self.arov_goal_sub

    def target_callback(self, msg):
        self.pose = msg.data

class ArovSelfNav(Node):
    def __init__(self):
        super().__init__('arov_self_pose')
        self.arov_self_sub = self.create_subscription(PoseStamped, 'topic', self.self_callback, 1)
        self.arov_self_sub

    def self_callback(self, msg):
        self.pose = msg.data

def main() -> None:
    rlcpy.init()
    self_pose = ArovSelfNav()
    rlcpy.spin(self_pose)
    target_pose = ArovTargetNav()
    rlcpy.spin(target_pose)

    initial_pose = self_pose.pose
    goal_pose = target_pose.pose
    goal_pose.pose.position.z = initial_pose.pose.position.z

    nav = BasicNavigator()
    nav.setInitialPose(initial_pose)
    nav.waitUntilNav2Active()
    goto_task = nav.goToPose(goal_pose)

    i = 0
    while not nav.isTaskComplete(task = goto_task):
        i += 1
        if i % 10 == 0:
            print('Moving to waypoint')

    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Waypoint reached')
    elif result == TaskResult.CANCELED:
        print('Movement cancelled')
    elif result == TaskResult.FAILED:
        (error_code, error_msg) = nav.getTaskError()
        print('Movement failed!{error_code}:{error_msg}')
    else:
        print('Movement has an invalid return status!')

    nav.lifecycleShutdown()

if __name__ == '__main__':
    main()
