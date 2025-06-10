from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PythonExpression, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='asv_arov_navigation').find('asv_arov_navigation')
    map = os.path.join(pkg_share, 'config', 'dog_pool.yaml')
    arov_params = os.path.join(pkg_share, 'config', 'arov_nav2_params.yaml')
    asv_params = os.path.join(pkg_share, 'config', 'asv_nav2_params.yaml')
    arov_urdf = os.path.join(pkg_share, 'models', 'arov_model.urdf')
    asv_urdf = os.path.join(pkg_share, 'models', 'asv_model.urdf')
    with open(arov_urdf, 'r') as infp:
        arov_description = infp.read()
    with open(asv_urdf, 'r') as infp:
        asv_description = infp.read()

    use_sim = LaunchConfiguration('use_sim')

    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            condition=IfCondition(use_sim)
        ),
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/kill ",
                "turtlesim/srv/Kill ",
                '"{name: turtle1}"',
            ]],
            shell=True,
            condition=IfCondition(use_sim)
        ),
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/spawn ",
                "turtlesim/srv/Spawn ",
                '"{x: 5.0, y: 5.0, theta: 0.0, name: arov}"',
            ]],
            shell=True,
            condition=IfCondition(use_sim)
        ),
        ExecuteProcess(
            cmd=[[
                FindExecutable(name='ros2'),
                " service call ",
                "/spawn ",
                "turtlesim/srv/Spawn ",
                '"{x: 6.0, y: 5.0, theta: 0.0, name: asv}"',
            ]],
            shell=True,
            condition=IfCondition(use_sim)
        ),
        Node(
            package='asv_arov_navigation',
            executable='asv_arov_control_server'
        ),
        Node(
            package='asv_arov_navigation',
            executable='movement_server'
        ),
        Node(
            namespace='arov',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': arov_description}]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'namespace': 'arov',
                'slam': 'false',
                'map': map,
                # 'keepout_mask': '',
                # 'speed_mask': '',
                'use_sim_time': 'false',
                'params_file': arov_params,
                'autostart': 'True',
                'use_composition': 'True',
                'use_respawn': 'false',
                'log_level': '0',
                'use_localization': 'false',
                'use_keepout_zones': 'false',
                'use_speed_zones': 'false'
            }.items(),
            #remappings=[('/cmd_vel'), ('/arov/cmd_vel')]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ),
            launch_arguments={
                'namespace': 'asv',
                'slam': 'false',
                'map': map,
                # 'keepout_mask': '',
                # 'speed_mask': '',
                'use_sim_time': 'false',
                'params_file': asv_params,
                'autostart': 'True',
                'use_composition': 'True',
                'use_respawn': 'false',
                'log_level': '0',
                'use_localization': 'false',
                'use_keepout_zones': 'false',
                'use_speed_zones': 'false'
            }.items(),
            #remappings=[('/cmd_vel'), ('/asv/cmd_vel')]
        )
    ])
