from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='asv_arov_navigation').find('asv_arov_navigation')
    map = os.path.join(pkg_share, 'config', 'dog_pool.yaml')
    arov_params = os.path.join(pkg_share, 'config', 'arov_nav2_dwb.yaml')
    asv_params = os.path.join(pkg_share, 'config', 'asv_nav2_dwb.yaml')
    control_server_params = os.path.join(pkg_share, 'config', 'control_server_params.yaml')
    arov_urdf = os.path.join(pkg_share, 'models', 'arov_model.urdf')
    asv_urdf = os.path.join(pkg_share, 'models', 'asv_model.urdf')
    arov_namespace = 'arov'
    asv_namespace = 'asv'

    use_sim = LaunchConfiguration('use_sim', default=False)

    ld = LaunchDescription([
        Node(
            package='asv_arov_navigation',
            executable='asv_arov_control_server',
            parameters=[control_server_params, {'use_sim': use_sim}]
        ),
        Node(
            package='asv_arov_navigation',
            executable='movement_server',
            parameters=[{'use_sim': use_sim}]
        ),
        Node(
            package='asv_arov_navigation',
            executable='arov_navigation_server'
        ),
        Node(
            namespace=arov_namespace,
            package='asv_arov_navigation',
            executable='lifecycle_pose_publisher',
            parameters=[{'cmd_vel_topic': '/arov/cmd_vel', 'use_sim': use_sim}]
        ),
        Node(
            namespace=asv_namespace,
            package='asv_arov_navigation',
            executable='lifecycle_pose_publisher',
            parameters=[{'cmd_vel_topic': '/asv/cmd_vel', 'use_sim': use_sim}]
        ),
        # Map server node
        Node(
            namespace=arov_namespace,
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[arov_params, {'yaml_filename': map}]
        ),
        # Nav2 Planner
        Node(
            namespace=arov_namespace,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[arov_params, {'use_sim_time': False}]
        ),
        # Nav2 Controller
        Node(
            namespace=arov_namespace,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[arov_params, {'use_sim_time': False,}]
        ),
        # Nav2 Behaviors
        Node(
            namespace=arov_namespace,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[arov_params, {'use_sim_time': False}]
        ),
        # Nav2 BT Navigator
        Node(
            namespace=arov_namespace,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[arov_params, {'map': map, 'use_sim_time': False}]
        ),
        # Map server node
        Node(
            namespace=asv_namespace,
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[asv_params, {'yaml_filename': map}]
        ),
        # Nav2 Planner
        Node(
            namespace=asv_namespace,
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[asv_params, {'use_sim_time': False}]
        ),
        # Nav2 Controller
        Node(
            namespace=asv_namespace,
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[asv_params, {'use_sim_time': False,}]
        ),
        # Nav2 Behaviors
        Node(
            namespace=asv_namespace,
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[asv_params, {'use_sim_time': False}]
        ),
        # Nav2 BT Navigator
        Node(
            namespace=asv_namespace,
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[asv_params, {'map': map, 'use_sim_time': False}]
        ),
        Node(
            namespace=arov_namespace,
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
            namespace=asv_namespace,
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
