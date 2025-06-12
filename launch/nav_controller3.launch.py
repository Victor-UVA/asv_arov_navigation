from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import PushRosNamespace

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='asv_arov_navigation').find('asv_arov_navigation')
    map = os.path.join(pkg_share, 'config', 'dog_pool.yaml')
    arov_params = os.path.join(pkg_share, 'config', 'arov_nav2_params.yaml')
    asv_params = os.path.join(pkg_share, 'config', 'asv_nav2_params.yaml')
    arov_urdf = os.path.join(pkg_share, 'models', 'arov_model.urdf')
    asv_urdf = os.path.join(pkg_share, 'models', 'asv_model.urdf')

    use_sim = LaunchConfiguration('use_sim')

    ld = LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            condition=IfCondition(use_sim),
            remappings=[('/arov/pose', '/arov/robot_pose'), ('/asv/pose', '/asv/robot_pose')]
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
            executable='asv_arov_control_server',
            parameters=[{'use_sim': use_sim}]
        ),
        Node(
            package='asv_arov_navigation',
            executable='movement_server',
            parameters=[{'use_sim': use_sim}]
        ),
        Node(
            namespace='arov',
            package='asv_arov_navigation',
            executable='lifecycle_pose_publisher',
            parameters=[{'cmd_vel_topic': '/arov/cmd_vel', 'use_sim': use_sim}]
        ),
        Node(
            namespace='asv',
            package='asv_arov_navigation',
            executable='lifecycle_pose_publisher',
            parameters=[{'cmd_vel_topic': '/asv/cmd_vel', 'use_sim': use_sim}]
        ),
        GroupAction(
            actions=[
                # Map server node
                PushRosNamespace('arov'),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[arov_params, {'yaml_filename': map}]
                ),
                # Nav2 Planner
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                ),
                # Nav2 Controller
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[arov_params, 
                                {'use_sim_time': False,
                                    'current_goal_checker': 'simple_goal_checker',
                                    'current_progress_checker': 'simple_progress_checker'
                                }]
                ),
                # Nav2 Behaviors
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                ),
                # Nav2 BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                )
            ]
        ),
        GroupAction(
            actions=[
                # Map server node
                PushRosNamespace('asv'),
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[arov_params, {'yaml_filename': map}]
                ),
                # Nav2 Planner
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                ),
                # Nav2 Controller
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[arov_params, 
                                {'use_sim_time': False,
                                    'current_goal_checker': 'simple_goal_checker',
                                    'current_progress_checker': 'simple_progress_checker'
                                }]
                ),
                # Nav2 Behaviors
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                ),
                # Nav2 BT Navigator
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[arov_params, {'use_sim_time': False}]
                )
            ]
        ),
        Node(
            namespace='arov',
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            namespace='asv',
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_odom',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ],
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            namespace="arov",
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0, # Fix to allow velocity integrator localization without bonding
                'node_names': ['map_server', 'pose_publisher', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        ),
        Node(
            namespace="asv",
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'bond_timeout': 0.0, # Fix to allow velocity integrator localization without bonding
                'node_names': ['map_server', 'pose_publisher', 'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']
            }]
        )
    ])

    return ld
