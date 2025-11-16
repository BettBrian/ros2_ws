#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # ICP Odometry - estimates odometry from lidar scans
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/MaxTranslation': '2.0',
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('scan_cloud', '/points')
        ]
    )

    # RTAB-Map SLAM node
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'use_sim_time': use_sim_time,
            'approx_sync': False,
            'wait_for_transform': 0.2,
            'Reg/Strategy': '1',  # 1=ICP
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlaneRadius': '0',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            'Odom/Strategy': '0',
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '30000',
            'Grid/ClusterRadius': '1',
            'Grid/RangeMax': '20',
            'Grid/RayTracing': 'true',
            'Grid/CellSize': '0.1',
            'Mem/NotLinkedNodesKept': 'false',
            'RGBD/ProximityMaxGraphDepth': '0',
            'RGBD/ProximityPathMaxNeighbors': '1',
            'RGBD/AngularUpdate': '0.05',
            'RGBD/LinearUpdate': '0.05',
            'Mem/STMSize': '30',
            'Rtabmap/DetectionRate': '2',
            'database_path': '~/ros2_ws/src/project/map/map.db',
        }],
        remappings=[
            ('scan_cloud', '/points'),
            ('odom', '/odom')
        ],
    )

    # Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': False
        }],
        remappings=[
            ('scan_cloud', '/points'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        icp_odometry,
        rtabmap_node,
        rtabmap_viz
    ])