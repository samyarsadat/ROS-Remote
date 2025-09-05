#!/bin/bash
ROS_DISTRO="jazzy"
NON_ROSDEP_DEPS="libxcb-cursor0 python3-lgpio python3-gpiozero"
set -e

sudo apt-get update && sudo apt-get install software-properties-common curl locales -y
sudo add-apt-repository universe -y && sudo apt-get update

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install ros-dev-tools ros-${ROS_DISTRO}-ros-base python3-pip -y
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

UDEV_RULES_FILE="/etc/udev/rules.d/50-ros-remote.rules"
VID_PID=("1fc9" "0001")
sudo tee "$UDEV_RULES_FILE" > /dev/null << EOF
# ROS Remote HID device rules
KERNEL=="hidraw*", SUBSYSTEM=="hidraw", ATTRS{idVendor}=="${VID_PID[0]}", ATTRS{idProduct}=="${VID_PID[1]}", TAG+="uaccess", MODE="660"
SUBSYSTEM=="usb", ATTR{idVendor}=="${VID_PID[0]}", ATTR{idProduct}=="${VID_PID[1]}", TAG+="uaccess", MODE="660"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger

cd "$HOME" || exit 1
git clone https://github.com/samyarsadat/ROS-Remote ./ros_remote --recurse-submodules
SOURCE_CODE_PATH="$HOME/ros_remote/source_code"
sudo chmod +s "$SOURCE_CODE_PATH/ros_ws_remote/deployment/run_gui.bash"

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