from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch.substitutions import PythonExpression, LaunchConfiguration, FindExecutable, Command
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

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
        # Map server node
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
        ),
        # Map server node
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[asv_params, {'yaml_filename': map}]
        ),
        # Nav2 Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[asv_params, {'use_sim_time': False}]
        ),
        # Nav2 Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[asv_params, 
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
            parameters=[asv_params, {'use_sim_time': False}]
        ),
        # Nav2 BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[asv_params, {'use_sim_time': False}]
        ),
    ])

    return ld
