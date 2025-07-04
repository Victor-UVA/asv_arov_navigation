import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Pose
from scipy.spatial.transform import Rotation
import numpy as np
import math

def build_pose(pose, orientation=None) :
    pose_msg = Pose()
    pose_msg.position.x = float(pose[0])
    pose_msg.position.y = float(pose[1])
    pose_msg.position.z = float(pose[2])
    pose_msg.orientation = quaternion_from_euler(pose[3:6]) if orientation is None else orientation
    return pose_msg

def build_pose_stamped(time, frame_id, pose, orientation=None) :
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = time.to_msg()
    pose_stamped.header.frame_id = frame_id
    pose_stamped.pose = build_pose(pose, orientation)
    return pose_stamped

def build_transform_stamped(time, parent_frame_id, child_frame_id, transform, orientation=None) :
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = time.to_msg()
    transform_stamped.header.frame_id = parent_frame_id
    transform_stamped.child_frame_id = child_frame_id
    transform_stamped.transform.translation.x = float(transform[0])
    transform_stamped.transform.translation.y = float(transform[1])
    transform_stamped.transform.translation.z = float(transform[2])
    transform_stamped.transform.rotation = quaternion_from_euler(transform[3:6]) if orientation is None else orientation
    return transform_stamped

def transform_pose_stamped(pose_stamped, transform) :

    p = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z], dtype=np.float64)
    r = Rotation.from_quat([pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w])
    R_mat = r.as_matrix() 
    # Build 4x4 homogeneous transform
    P_april = np.eye(4)
    P_april[0:3, 0:3] = R_mat
    P_april[0:3, 3] = p

    # Extract translation
    t = transform.transform.translation
    trans = np.array([t.x, t.y, t.z], dtype=np.float64)

    # Extract rotation matrix from quaternion
    q = transform.transform.rotation
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
    R_mat = rot.as_matrix()  # 3x3

    # Build 4x4 homogeneous transformation matrix
    T = np.eye(4)
    T[0:3, 0:3] = R_mat
    T[0:3, 3] = trans

    P_map = T@P_april

    # Extract translation
    trans = P_map[0:3, 3]

    # Extract rotation matrix and convert to quaternion
    rot_matrix = P_map[0:3, 0:3]
    quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

    # Create PoseStamped
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = trans[0]
    pose.pose.position.y = trans[1]
    pose.pose.position.z = trans[2]
    pose.pose.orientation.x = quat[0]
    pose.pose.orientation.y = quat[1]
    pose.pose.orientation.z = quat[2]
    pose.pose.orientation.w = quat[3]
   
    return pose

def transform_pose(pose_old, transform) :
    p = np.array([pose_old.position.x, pose_old.position.y, pose_old.position.z], dtype=np.float64)
    r = Rotation.from_quat([pose_old.orientation.x, pose_old.orientation.y, pose_old.orientation.z, pose_old.orientation.w])
    R_mat = r.as_matrix() 
    # Build 4x4 homogeneous transform
    P_april = np.eye(4)
    P_april[0:3, 0:3] = R_mat
    P_april[0:3, 3] = p

    # Extract translation
    t = transform.transform.translation
    trans = np.array([t.x, t.y, t.z], dtype=np.float64)

    # Extract rotation matrix from quaternion
    q = transform.transform.rotation
    rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
    R_mat = rot.as_matrix()  # 3x3

    # Build 4x4 homogeneous transformation matrix
    T = np.eye(4)
    T[0:3, 0:3] = R_mat
    T[0:3, 3] = trans

    P_map = T @ P_april

    # Extract translation
    trans = P_map[0:3, 3]

    # Extract rotation matrix and convert to quaternion
    rot_matrix = P_map[0:3, 0:3]
    quat = Rotation.from_matrix(rot_matrix).as_quat()  # [x, y, z, w]

    # Create Pose
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

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

def normalize_angle(theta) :
    return normalize(theta, -math.pi, math.pi)

def normalize(x, min, max) :
    return (x - min) % (max - min) + min
