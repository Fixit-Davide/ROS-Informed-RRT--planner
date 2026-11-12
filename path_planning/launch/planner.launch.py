import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('path_planning')
    params = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'planner.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('auto_arm', default_value='false',
                              description='Let the planner switch PX4 to OFFBOARD and arm'),
        DeclareLaunchArgument('mapping', default_value='true',
                              description='Also start voxel filter + octomap_server'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapping.launch.py')),
            condition=IfCondition(LaunchConfiguration('mapping')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(
            package='path_planning',
            executable='path_planning_node',
            name='path_planning',
            output='screen',
            parameters=[
                params,
                {
                    'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                    'control.auto_arm': ParameterValue(LaunchConfiguration('auto_arm'),
                                                       value_type=bool),
                },
            ],
        ),
    ])
