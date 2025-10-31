# my_robot/launch/rsp.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # Declare launch argument for use_ros2_control
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Use ros2_control hardware interface (true/false)'
    )

    # Path to your Xacro file
    xacro_file = os.path.join(
        get_package_share_directory('my_robot'),
        'urdf', 'robot.urdf.xacro'
    )

    # Process the Xacro file with use_ros2_control parameter
    try:
        robot_desc = xacro.process_file(xacro_file, mappings={'use_ros2_control': use_ros2_control}).toxml()
    except Exception as e:
        print(f"Error processing Xacro file: {e}")
        raise

    return LaunchDescription([
        use_ros2_control_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': LaunchConfiguration('use_sim_time', default='true')
            }]
        )
    ])