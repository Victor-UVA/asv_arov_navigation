import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator
from asv_arov_navigation.between_fence_navigation_server import NavigationActionServer
from robot_guidance_pkg.apriltag_navigation_server import AprilTagNavigation

def main(args=None) :
    rclpy.init()
    arov_navigator = BasicNavigator("arov")
    arov_navigator.waitUntilNav2Active(localizer="/arov/pose_publisher")
    between_fence_navigation_server = NavigationActionServer(arov_navigator)
    cleaning_navigation_server = AprilTagNavigation(arov_navigator)
    executor = SingleThreadedExecutor()
    executor.add_node(between_fence_navigation_server)
    executor.add_node(cleaning_navigation_server)
    executor.spin()

if __name__ == '__main__' :
    main()
