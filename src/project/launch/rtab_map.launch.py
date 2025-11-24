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

    # ICP Odometry 
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'baselink_link',
            'odom_frame_id': 'odom',
            'use_sim_time': use_sim_time,
            'wait_for_transform': 0.2,
            'expected_update_rate': 15.0,
            
            # ICP CONFIGURATION
            'Icp/PointToPlane': 'true',
            'Icp/PointToPlaneK': '20',
            'Icp/VoxelSize': '0.1',
            'Icp/MaxCorrespondenceDistance': '1.0',
            'Icp/MaxTranslation': '2.0',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            
            'Odom/Strategy': '0',
            'Odom/GuessMotion': 'true',
            'Odom/ResetCountdown': '1',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points') 
        ]
    )


    # RTAB-Map SLAM 
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'baselink_link',
            'map_frame_id': 'map',
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan_cloud': True,
            'use_sim_time': use_sim_time,
            'approx_sync': True,
            'wait_for_transform': 0.2,
            
            # ICP / REGISTRATION
            'Reg/Strategy': '1', 
            'Icp/VoxelSize': '0.1',
            'Icp/PointToPlaneK': '20',
            'Icp/PointToPlane': 'true',
            'Icp/Iterations': '10',
            'Icp/Epsilon': '0.001',
            'Icp/MaxTranslation': '3',
            'Icp/MaxCorrespondenceDistance': '1',
            'Icp/Strategy': '1',
            'Icp/OutlierRatio': '0.7',
            'Icp/CorrespondenceRatio': '0.01',
            
            # POINT CLOUD SPECIFIC
            'OdomF2M/ScanSubtractRadius': '0.1',
            'OdomF2M/ScanMaxSize': '30000',
            'Grid/RangeMax': '20',
            'Grid/RayTracing': 'true',
            'Grid/CellSize': '0.1',
            'Grid/3D': 'true', # Generate 3D Octomap
            
            # MEMORY
            'Mem/NotLinkedNodesKept': 'false',
            'Mem/STMSize': '30',
            'Rtabmap/DetectionRate': '1', # Don't update map too fast
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('odom', '/odom')
        ],
        arguments=['--delete_db_on_start']
    )

    # Visualization (RTAB-Map Viz)
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'use_sim_time': use_sim_time,
            'subscribe_scan_cloud': True,
            'subscribe_odom_info': True,
            'approx_sync': True
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            ('odom', '/odom')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        icp_odometry,
        rtabmap_node,
        rtabmap_viz
    ])