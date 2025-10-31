#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('project')

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true'
    )

    rtabmap_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'approx_sync': False,
            'queue_size': 10,
            'Grid/3D': True,
            'Grid/RangeMax': 30.0,
            'Grid/FromDepth': False,
            'RGBD/NeighborLinkRefining': True,
            'Reg/Strategy': '1',  # ICP
            'Vis/MinInliers': 10,
            'Rtabmap/DetectionRate': 1.0,
        }],
        remappings=[
            ('scan_cloud', '/points'),
            ('odom', '/odom'),
            ('map', '/rtabmap/map'),
            ('grid_map', '/rtabmap/grid_map'),
        ],
        arguments=[
            '--delete_db_on_start'
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        rtabmap_node
    ])