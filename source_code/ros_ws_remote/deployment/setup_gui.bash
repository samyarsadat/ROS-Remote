#!/bin/bash
ROS_DISTRO="jazzy"
NON_ROSDEP_DEPS="libxcb-cursor0 python3-lgpio python3-gpiozero"
set -e

sudo apt-get install software-properties-common
sudo add-apt-repository universe -y
sudo apt-get update && sudo apt-get install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo curl -sSL https://raw.githubusercontent.com/eProsima/vulcanexus/main/vulcanexus.key -o /usr/share/keyrings/vulcanexus-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/vulcanexus-archive-keyring.gpg] http://repo.vulcanexus.org/debian $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/vulcanexus.list > /dev/null

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install vulcanexus-$ROS_DISTRO-base python3-pip -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo rosdep init
rosdep update

sudo usermod -aG dialout $USER

CONFIG_FILE="/boot/firmware/config.txt"
DTOV_EXISTS=$(grep -q "^dtoverlay=disable-bt" "$CONFIG_FILE" && echo "1" || echo "0")
ENUA_EXISTS=$(grep -q "^enable_uart=1" "$CONFIG_FILE" && echo "1" || echo "0")

if grep -q "^\[all\]" "$CONFIG_FILE"; then
    if [ "$DTOV_EXISTS" -eq 0 ]; then
        sudo sed -i '0,/^\[all\]/s//[all]\ndtoverlay=disable-bt/' "$CONFIG_FILE"
    fi
    
    if [ "$ENUA_EXISTS" -eq 0 ]; then
        sudo sed -i '0,/^\[all\]/s//[all]\nenable_uart=1/' "$CONFIG_FILE"
    fi
else
    echo -e "\n[all]" | sudo tee -a "$CONFIG_FILE" > /dev/null
    [ "$DTOV_EXISTS" -eq 0 ] && echo "dtoverlay=disable-bt" | sudo tee -a "$CONFIG_FILE" > /dev/null
    [ "$ENUA_EXISTS" -eq 0 ] && echo "enable_uart=1" | sudo tee -a "$CONFIG_FILE" > /dev/null
fi

cd "$HOME" || exit 1
git clone https://github.com/samyarsadat/ROS-Remote ./ros_remote --recurse-submodules
SOURCE_CODE_PATH="$HOME/ros_remote/source_code"
sudo chmod +s "$SOURCE_CODE_PATH/ros_ws_remote/deployment/run_gui.bash"

cd "$SOURCE_CODE_PATH/pico_ws/libmicroros" || exit 1
colcon build --packages-select remote_pico_coms
source "./install/local_setup.sh"

cd "$SOURCE_CODE_PATH/ros_robot_project/source_code/ros_ws_robot_infra" || exit 1
colcon build --packages-select ros_robot_msgs
source "./install/local_setup.sh"

cd "$SOURCE_CODE_PATH/ros_ws_remote" || exit 1
export PIP_BREAK_SYSTEM_PACKAGES=1
rosdep install --from-paths src -y --ignore-src
sudo apt-get install $NON_ROSDEP_DEPS -y
colcon build --packages-skip test_camera_publisher

sudo apt-get autoremove -y
sudo apt-get autoclean -y