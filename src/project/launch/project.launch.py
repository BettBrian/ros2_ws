import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'project'
    
    # Get package paths
    pkg_share_path = get_package_share_directory(package_name)
    
    # Set Gazebo resource paths
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([
            pkg_share_path,
            os.path.join(pkg_share_path, 'meshes'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ])
    )
    
    pkg_share = FindPackageShare(package_name)
    world_path = PathJoinSubstitution([pkg_share, 'world', 'empty.sdf'])
    controllers_path = PathJoinSubstitution([pkg_share, 'config', 'controllers.yaml'])
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    
    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'rsp.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': 'true'
        }.items()
    )
    
    # Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ', world_path]}.items()
    )
    
    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'quadped',
            '-z', '0.3'
        ],
        output='screen'
    )
    
    # Gazebo Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        parameters=[{'use_sim_ time': use_sim_time}],
        output='screen'

    )

    # Joint State Broadcaster Spawner (DELAYED)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )
    
    # Position Controller Spawner (DELAYED)
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'position_controller',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '120'
        ],
        output='screen'
    )
    
    # Delay spawners
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[position_controller_spawner],
        )
    )
    
    return LaunchDescription([
        gz_resource_path,
        declare_use_sim_time,
        rsp,
        gazebo_launch,
        spawn_entity,
        gz_bridge,
        delayed_joint_state_broadcaster,
        delayed_position_controller,
    ])