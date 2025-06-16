import rclpy
from robot_guidance.robot_guidance_pkg import ApriltagNavigationClient
from robot_guidance.robot_guidance_interfaces.action import NavigateAprilTags
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation

def setup_pose_stamped(header, pose) :
    pose = PoseStamped()
    pose.header = header
    pose.pose.position.x = pose[0]
    pose.pose.position.y = pose[1]
    pose.pose.position.z = pose[2]
    quat = Rotation.from_euler("xyz", pose[3:6], degrees=True).as_quat()
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
    return pose

def main(args=None) :
    rclpy.init()
    guidance_client = ApriltagNavigationClient()
    header = Header()
    header.frame_id = "map"
    header.stamp = guidance_client.get_clock().now().to_msg()
    pose1 = setup_pose_stamped(header, [0, 0, 0, 0, 0, 0])
    pose2 = setup_pose_stamped(header, [0, 0, 3, 0, 0, 0])
    pose3 = setup_pose_stamped(header, [-1, 0, 3, 0, 0, 0])
    pose4 = setup_pose_stamped(header, [-1, 0, 0, 0, 0, 0])
    future = guidance_client.send_goal([pose1, pose2, pose3, pose4], ["vertical", "right", "vertical"])
    rclpy.spin_until_future_complete(guidance_client, future)

if __name__ == '__main__' :
    main()