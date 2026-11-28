"""What the map server needs: the map -> base_link -> lidar_link tf chain.

The map server builds the occupancy map (Bonxai) from the cloud on its `topics.cloud`
parameter. The map -> base_link tf comes from the drone_pose package (PX4 odometry), in the
same NWU convention as the controller server's default `frame_convention`. In simulation,
`gz_bridge:=true` also bridges the Gazebo lidar cloud and /clock into ROS (needs
use_sim_time:=true, since the cloud is stamped with simulation time).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    sim = {'use_sim_time': use_sim_time}
    cloud_topic = LaunchConfiguration('cloud_topic')
    base_frame = LaunchConfiguration('base_frame')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/points'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar_link',
                              description='frame_id of the point cloud'),
        # Must match the lidar_joint pose in sim/models/x500_lidar_3d/model.sdf.
        DeclareLaunchArgument('lidar_z', default_value='0.30',
                              description='Lidar height above base_link [m]'),
        DeclareLaunchArgument('drone_pose', default_value='true',
                              description='Start drone_pose (map -> base_link tf from PX4)'),
        DeclareLaunchArgument('gz_bridge', default_value='false',
                              description='Bridge the Gazebo lidar cloud and /clock'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gz_bridge')),
            arguments=[
                ['/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            remappings=[('/lidar/points', cloud_topic)],
            parameters=[sim],
        ),

        Node(
            package='drone_pose',
            executable='DronePoseExecutable',
            name='drone_pose',
            output='screen',
            condition=IfCondition(LaunchConfiguration('drone_pose')),
            parameters=[{'base_frame': 'map',
                         'child_frame': base_frame,
                         'pub.pose': True, 'pub.path': True, 'pub.vel': False}, sim],
        ),

        # base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['--x', '0', '--y', '0', '--z', LaunchConfiguration('lidar_z'),
                       '--frame-id', base_frame,
                       '--child-frame-id', lidar_frame],
            parameters=[sim],
        ),
    ])
