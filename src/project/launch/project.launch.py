#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler

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

    # 1. Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'rsp.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items()
    )

    # 3. Spawn
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'quad', '-z', '0.5'],
        output='screen'
    )

    # 4. Gazebo Bridge (FIXED: parameters in `parameters=`)
    gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V', 
    ],
    remappings=[
        ('/scan/points', '/points'),
        ('/odometry', '/odom'),
        ('/tf', '/tf'),
    ],
    parameters=[{'use_sim_time': use_sim_time}],
    output='screen'
    )
    # Delay bridge until spawn is ready
    delay_gz_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[gz_bridge]
        )
    )

    fix_lidar_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='fix_lidar_frame',
    arguments=[
        '0', '0', '0',   # x y z
        '0', '0', '0',   # qx qy qz
        '1',             # qw
        'lidar_link',    # parent
        'quad/base_link/lidar'  # child
    ],
    output='screen'
    )

    # 5. RTAB-Map
    rtabmap_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg_share, 'launch', 'rtab_map.launch.py')
    ),
    launch_arguments={'use_sim_time': use_sim_time}.items()
)

    return LaunchDescription([
        declare_use_sim_time,
        rsp,
        gazebo_launch,
        spawn_entity,
        delay_gz_bridge,
        fix_lidar_tf,
        rtabmap_launch
    ])