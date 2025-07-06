import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() :

    use_sim = LaunchConfiguration('use_sim', default=False)

    return LaunchDescription([
        Node(
            package='asv_arov_navigation',
            executable='nav0',
            parameters=[{'use_sim': use_sim}]
        ),
        Node(
            package='asv_arov_navigation',
            executable='nav0_sim',
            parameters=[{'use_sim': use_sim}]
        )
    ])
