from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Orientation
from scipy.spatial.transform import Rotation
import numpy as np

def build_pose_stamped(time, frame_id, pose, orientation=None) :
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = time.to_msg()
    pose_stamped.header.frame_id = frame_id
    if orientation is None :
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        pose_stamped.pose.orientation = quaternion_from_euler(pose[3:6])
    else :
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = pose[2]
        pose_stamped.pose.orientation = orientation
    return pose_stamped

def quaternion_from_euler(rpy) :
    quat = Rotation.from_euler("xyz", rpy, degrees=False).as_quat()
    out = Orientation()
    out.x = quat[0]
    out.y = quat[1]
    out.z = quat[2]
    out.w = quat[3]
    return out

def euler_from_quaternion(quat) :
    return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz", degrees=False)