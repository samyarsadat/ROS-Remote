#!/bin/bash
set -e

sudo apt install software-properties-common
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Temporary Vulcanexus installation.
sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null
sudo apt update & sudo apt install -y vulcanexus-humble-base

sudo apt update && sudo apt upgrade -y
sudo apt-get install ros-humble-desktop ros-dev-tools libxcb-cursor0 python3-pip python3-lgpio python3-gpiozero -y
pip install PySide6
source /opt/ros/humble/setup.bash

cd "$HOME" || exit 1
git clone https://github.com/samyarsadat/ROS-Remote ./ros_remote --recurse-submodules
cd ./ros_remote || exit 1
sudo chmod +s "./source_code/ros_ws_remote/deployment/run_gui.bash"
cd "./source_code/ros_ws_remote" || exit 1
colcon build --packages-skip test_camera_publisher
cd "../pico_ws/libmicroros" || exit 1
colcon build --packages-select remote_pico_coms
cd "../../ros_robot_project/source_code/ros_ws_robot" || exit 1
colcon build --packages-select ros_robot_msgs