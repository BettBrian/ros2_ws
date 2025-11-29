import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get paths
    pkg_share = get_package_share_directory('project')
    urdf_file = os.path.join(pkg_share, 'urdf', 'quadped.urdf')
    
    # Read URDF 
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    
    robot_desc = robot_desc.replace('$(find project)', pkg_share)
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_use_ros2_control_cmd = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use ros2_control if true'
    )
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc
            }
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_use_ros2_control_cmd,
        robot_state_publisher_node
    ])