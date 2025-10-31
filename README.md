# ROS 2 Workspace

## Setup

```bash
# Install ROS 2 Jazzy
sudo apt update && sudo apt install -y python3-colcon-common-extensions ros-jazzy-desktop

# Clone and build
git clone https://github.com/BettBrian/ros2_ws.git
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Source
source install/setup.bash

# Run example
ros2 run project project.launch.py
