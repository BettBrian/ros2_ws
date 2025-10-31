import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


# CRITICAL: Import xacro here so it's available in PythonExpression
import xacro


def generate_launch_description():
    # ------------------------------------------------------------------
    # 1. Launch arguments
    # ------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Enable ros2_control hardware interface (true/false)'
    )

    # ------------------------------------------------------------------
    # 2. Xacro file path
    # ------------------------------------------------------------------
    xacro_path = os.path.join(
        get_package_share_directory('project'),
        'urdf',
        'quad.urdf.xacro'
    )

    # ------------------------------------------------------------------
    # 3. Process Xacro at launch time
    # ------------------------------------------------------------------
    try:
        robot_description = xacro.process_file(xacro_path, mappings={'use_ros2_control': use_ros2_control}).toxml()
    except Exception as e:
        print(f"Error processing Xacro file: {e}")
        raise

    # ------------------------------------------------------------------
    # 4. Robot State Publisher
    # ------------------------------------------------------------------
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # ------------------------------------------------------------------
    # 5. Return
    # ------------------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_ros2_control,
        rsp_node
    ])