import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() :

    nav_pkg_share = FindPackageShare(package='asv_arov_navigation').find('asv_arov_navigation')
    cleaning_pkg_share = FindPackageShare(package='robot_guidance').find('robot_guidance')

    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_pkg_share, 'launch', 'nav_controller_launch.py')
            ),
            launch_arguments={
                'use_sim': use_sim
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(cleaning_pkg_share, 'launch', 'cleaner_launch.py')
            )
        )
    ])
