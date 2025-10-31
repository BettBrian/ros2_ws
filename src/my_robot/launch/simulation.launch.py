import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_robot'

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Include the robot state publisher launch file
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Define the default path to the world file
    default_world = os.path.join(
        get_package_share_directory('ros_gz_sim'),
        'worlds', 'world.world'
    )

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='Full path to the world model file to load'
    )

    # Start the Gazebo server
    gzserver_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world],
        output='screen'
    )

    # Start the Gazebo client
    gzclient_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen'
    )

    # Run the spawner node from ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'robot',
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for cmd_vel, odom, and scan
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/my_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/my_robot/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/my_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )

    # Controller manager for ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_controller_manager',
        parameters=[os.path.join(
            get_package_share_directory(package_name),
            'config', 'controller_config.yaml'
        )],
        output='screen'
    )

    # Delay spawn_entity, gz_bridge, and controller_manager until gzserver has started
    delay_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver_cmd,
            on_start=[spawn_entity]
        )
    )

    delay_gz_bridge = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver_cmd,
            on_start=[gz_bridge]
        )
    )

    delay_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=gzserver_cmd,
            on_start=[controller_manager]
        )
    )

    # Launch controllers after controller_manager
    delay_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller'],
                    output='screen'
                )
            ]
        )
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        rsp,
        world_arg,
        gzserver_cmd,
        gzclient_cmd,
        delay_spawn_entity,
        delay_gz_bridge,
        delay_controller_manager,
        delay_diff_drive_controller,
        delay_joint_state_broadcaster,
    ])