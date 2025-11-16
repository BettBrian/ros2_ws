#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'project'
    pkg_share = get_package_share_directory(package_name)
    world_path = os.path.join(pkg_share, 'world', 'empty.world')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'use_ros2_control': 'false'
                          }.items()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'quad', '-z', '0.5'],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/quad/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/quad/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/world/test_industry/model/quad/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/scan/points', '/points'),
            ('/world/test_industry/model/quad/joint_state', '/joint_states'),
            ('/model/quad/odometry', '/odom'),
            ('/model/quad/tf', '/tf')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    fix_lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='fix_lidar_frame',
    arguments=[
        '0', '0', '0',   # x y z
        '0', '0', '0',   # qx qy qz
        '1',             # qw
        'odom',    # parent
        'quad/base_link/lidar'  # child
    ],
    output='screen'
    )


    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        gazebo_launch,
        spawn_entity,
        gz_bridge,
        fix_lidar_tf
    ])