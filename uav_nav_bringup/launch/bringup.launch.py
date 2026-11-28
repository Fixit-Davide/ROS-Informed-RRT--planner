"""UAV navigation servers: map, planner, controller, navigator + their lifecycle manager.

use_composition:=true (default) loads the four servers into one component container with
intra-process comms, so the map server's voxel_map snapshots reach the planner and controller
servers without being copied or serialized. use_composition:=false runs each server in its
own process (the map then travels as uav_nav_msgs/VoxelMap messages), useful to debug or
restart one server alone.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

# (package, component plugin, executable, node name), in lifecycle start-up order.
SERVERS = [
    ('uav_map_server', 'uav_map_server::MapServer', 'map_server', 'map_server'),
    ('uav_planner_server', 'uav_planner_server::PlannerServer', 'planner_server', 'planner_server'),
    ('uav_controller_server', 'uav_controller_server::ControllerServer', 'controller_server',
     'controller_server'),
    ('uav_navigator', 'uav_navigator::Navigator', 'navigator', 'navigator'),
]


def generate_launch_description():
    share = get_package_share_directory('uav_nav_bringup')
    params = LaunchConfiguration('params_file')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    overrides = {
        'use_sim_time': use_sim_time,
        'auto_arm': ParameterValue(LaunchConfiguration('auto_arm'), value_type=bool),
    }
    server_params = [params, overrides]
    log_args = ['--ros-args', '--log-level', LaunchConfiguration('log_level')]

    composed = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        # One executor thread per server: a planner solve or a map insert never delays the
        # controller's setpoint stream.
        arguments=['--executor-type', 'single-threaded', '--isolated'] + log_args,
        output='screen',
        composable_node_descriptions=[
            ComposableNode(package=pkg, plugin=plugin, name=name, parameters=server_params,
                           extra_arguments=[{'use_intra_process_comms': True}])
            for pkg, plugin, _, name in SERVERS
        ],
        condition=IfCondition(LaunchConfiguration('use_composition')),
    )
    separate = GroupAction(
        [Node(package=pkg, executable=exe, name=name, output='screen',
              parameters=server_params, arguments=log_args)
         for pkg, _, exe, name in SERVERS],
        condition=UnlessCondition(LaunchConfiguration('use_composition')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='Let the controller switch PX4 to OFFBOARD and arm'),
        DeclareLaunchArgument('use_composition', default_value='true',
                              description='One container, zero-copy intra-process map'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='Configure and activate the servers at start-up'),
        DeclareLaunchArgument('mapping', default_value='true',
                              description='Also start drone_pose and the lidar tf (mapping.launch.py)'),
        DeclareLaunchArgument('gz_bridge', default_value='false',
                              description='Bridge the Gazebo lidar cloud and /clock (SITL)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Open RViz2 with the navigation view (config/nav.rviz)'),
        DeclareLaunchArgument('log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapping.launch.py')),
            condition=IfCondition(LaunchConfiguration('mapping')),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
                              'gz_bridge': LaunchConfiguration('gz_bridge')}.items(),
        ),

        composed,
        separate,

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[params, {
                'use_sim_time': use_sim_time,
                'autostart': ParameterValue(LaunchConfiguration('autostart'), value_type=bool),
                'node_names': [name for _, _, _, name in SERVERS],
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'config', 'nav.rviz')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
