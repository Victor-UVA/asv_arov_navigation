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

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'route_server',
        'behavior_server',
        'velocity_smoother',
        'collision_monitor',
        'bt_navigator',
        'waypoint_follower',
        'docking_server',
    ]

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    configured_asv_params = ParameterFile(
        RewrittenYaml(
            source_file=asv_params,
            root_key='asv',
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_arov_params = ParameterFile(
        RewrittenYaml(
            source_file=arov_params,
            root_key='arov',
            convert_types=True,
        ),
        allow_substs=True,
    )

    load_composable_nodes_asv = GroupAction(
        actions=[
            LoadComposableNodes(
                target_container='asv',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_asv_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_asv_params, {'graph_filepath': ''}],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_asv_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_asv_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_asv_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': 'true', 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

    load_composable_nodes_arov = GroupAction(
        actions=[
            LoadComposableNodes(
                target_container='asv',
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_controller',
                        plugin='nav2_controller::ControllerServer',
                        name='controller_server',
                        parameters=[configured_arov_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_smoother',
                        plugin='nav2_smoother::SmootherServer',
                        name='smoother_server',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_planner',
                        plugin='nav2_planner::PlannerServer',
                        name='planner_server',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_route',
                        plugin='nav2_route::RouteServer',
                        name='route_server',
                        parameters=[configured_arov_params, {'graph_filepath': ''}],
                        remappings=remappings),
                    ComposableNode(
                        package='nav2_behaviors',
                        plugin='behavior_server::BehaviorServer',
                        name='behavior_server',
                        parameters=[configured_arov_params],
                        remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_bt_navigator',
                        plugin='nav2_bt_navigator::BtNavigator',
                        name='bt_navigator',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_waypoint_follower',
                        plugin='nav2_waypoint_follower::WaypointFollower',
                        name='waypoint_follower',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_velocity_smoother',
                        plugin='nav2_velocity_smoother::VelocitySmoother',
                        name='velocity_smoother',
                        parameters=[configured_arov_params],
                        remappings=remappings
                        + [('cmd_vel', 'cmd_vel_nav')],
                    ),
                    ComposableNode(
                        package='nav2_collision_monitor',
                        plugin='nav2_collision_monitor::CollisionMonitor',
                        name='collision_monitor',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='opennav_docking',
                        plugin='opennav_docking::DockingServer',
                        name='docking_server',
                        parameters=[configured_arov_params],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_navigation',
                        parameters=[
                            {'autostart': 'true', 'node_names': lifecycle_nodes}
                        ],
                    ),
                ],
            ),
        ],
    )

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
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', arov_urdf])}],
            condition=IfCondition(PythonExpression(['not ', use_sim]))
        ),
        Node(
            namespace='asv',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', asv_urdf])}],
            condition=IfCondition(PythonExpression(['not ', use_sim]))
        ),
    ])

    ld.add_action(load_composable_nodes_asv)
    ld.add_action(load_composable_nodes_arov)

    return ld
