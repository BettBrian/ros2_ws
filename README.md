# Ensure you have
* **OS:** Ubuntu 24.04 (Noble Numbat)
* **ROS Distro:** ROS 2 Jazzy 
* **Gazebo:** Harmonic

# Dependencies
Install the required ROS 2 packages :

sudo apt update
sudo apt install ros-jazzy-ros-gz \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control \
                 ros-jazzy-rtabmap-rosv


# Clone and build
git clone https://github.com/BettBrian/ros2_ws.git
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source
source install/setup.bash

# Run main lauch
ros2 run project project.launch.py

# Run rtab_map
ros2 launch project rtabmap.launch.py


