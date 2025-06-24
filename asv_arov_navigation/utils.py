import rclpy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
import numpy as np

def build_pose_stamped(time, frame_id, pose, orientation=None) :
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = time.to_msg()
    pose_stamped.header.frame_id = frame_id
    if orientation is None :
        pose_stamped.pose.position.x = float(pose[0])
        pose_stamped.pose.position.y = float(pose[1])
        pose_stamped.pose.position.z = float(pose[2])
        pose_stamped.pose.orientation = quaternion_from_euler(pose[3:6])
    else :
        pose_stamped.pose.position.x = float(pose[0])
        pose_stamped.pose.position.y = float(pose[1])
        pose_stamped.pose.position.z = float(pose[2])
        pose_stamped.pose.orientation = orientation
    return pose_stamped

def build_transform_stamped(time, parent_frame_id, child_frame_id, transform) :
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = time.to_msg()
    transform_stamped.header.frame_id = parent_frame_id
    transform_stamped.child_frame_id = child_frame_id
    transform_stamped.transform.translation.x = float(transform[0])
    transform_stamped.transform.translation.y = float(transform[1])
    transform_stamped.transform.translation.z = float(transform[2])
    transform_stamped.transform.rotation = quaternion_from_euler(transform[3:6])
    return transform_stamped

def transform_pose_stamped(pose_stamped, transform_stamped) :
    xl = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y, 0], dtype=np.float64)
    R = Rotation.from_quat([transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w])
    xa = R.apply(xl)
    out = PoseStamped()
    out.pose.position.x = xa[0] + transform_stamped.transform.translation.x
    out.pose.position.y = xa[1] + transform_stamped.transform.translation.y
    out.pose.position.z = pose_stamped.pose.position.z + transform_stamped.transform.translation.z
    Q = Rotation.from_quat([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
    quat = (Q * R).as_quat()
    out.pose.orientation.x = quat[0]
    out.pose.orientation.y = quat[1]
    out.pose.orientation.z = quat[2]
    out.pose.orientation.w = quat[3]
    return out

def quaternion_from_euler(rpy) :
    quat = Rotation.from_euler("xyz", rpy, degrees=False).as_quat()
    out = Quaternion()
    out.x = float(quat[0])
    out.y = float(quat[1])
    out.z = float(quat[2])
    out.w = float(quat[3])
    return out

def euler_from_quaternion(quat) :
    return Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler("xyz", degrees=False)
