import rlcpy
from rlcpy.duration import Duration
from rlcpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool, Float32


def main() -> None:
    rlcpy.init()

    initial_pose = PoseStamped()
    initial_pose.header.frame_id = "map"
    initial_pose.header.stamp = self.get_clock().now()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = 0.0
    initial_pose.pose.orientation.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.header.stamp = self.get_clock().now()
    goal_pose.pose.position.x = 10.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    nav = BasicNavigator('arov/base_link')
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