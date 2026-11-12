"""Point cloud -> voxel filter -> octomap_server (replaces ROS 1 mavros_mapping.launch).

The map -> base_link tf is expected from the drone_pose package (PX4 odometry), which
uses the same NWU convention as the planner's default `frame_convention`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('path_planning')
    params = os.path.join(share, 'config', 'mapping.yaml')
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    sim = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/camera/depth/color/points'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('drone_pose', default_value='true',
                              description='Start drone_pose (map -> base_link tf from PX4)'),

        Node(
            package='drone_pose',
            executable='DronePoseExecutable',
            name='drone_pose',
            output='screen',
            condition=IfCondition(LaunchConfiguration('drone_pose')),
            parameters=[{'base_frame': 'map',
                         'child_frame': LaunchConfiguration('base_frame'),
                         'pub.pose': True, 'pub.path': True, 'pub.vel': False}, sim],
        ),

        # base_link -> camera_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf',
            arguments=['--x', '0.1', '--y', '0', '--z', '0',
                       '--frame-id', LaunchConfiguration('base_frame'),
                       '--child-frame-id', LaunchConfiguration('camera_frame')],
            parameters=[sim],
        ),

        ComposableNodeContainer(
            name='pcl_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::VoxelGrid',
                    name='voxel_grid',
                    parameters=[params, sim],
                    remappings=[('input', LaunchConfiguration('cloud_topic')),
                                ('output', '/voxel_grid/output')],
                ),
            ],
        ),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[params, sim],
            remappings=[('cloud_in', '/voxel_grid/output')],
        ),
    ])
