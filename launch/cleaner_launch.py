import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description() :
    pkg_path = get_package_share_directory('robot_guidance_pkg')
    depth_control_param_file = os.path.join(pkg_path, 'config', 'depth_control_params.yaml')
    strafe_control_param_file = os.path.join(pkg_path, 'config', 'strafe_control_params.yaml')

    return LaunchDescription([
        Node(
            package='robot_guidance_pkg',
            executable='depth_control_server',
            name='depth_control_server',
            parameters=[
                depth_control_param_file,
                {
                    'cmd_vel_topic': '/arov/cmd_vel',
                    'odom_topic': '/arov/odom'
                }
            ],
        ),
        Node(
            package='robot_guidance_pkg',
            executable='strafe_control_server',
            name='strafe_control_server',
            parameters=[
                strafe_control_param_file,
                {
                    'cmd_vel_topic': '/arov/cmd_vel',
                    'odom_topic': '/arov/odom'
                }
            ],
        )
    ])
