#!/usr/bin/env bash
# Devcontainer post-create script.
# This script is run after the devcontainer is created.

set -e
echo "--> post_create.sh started!"
echo "-> First container run, running setup..."

echo "-> Setting folder permissions and copying files..."
sudo chown -R nonroot: $HOME/ros_ws/
    
#echo "-> Installing MicroROS tools..."
#sudo apt-get update \
#&& rosdep update \
#&& cd $HOME/pico_ws/libmicroros \
#&& rosdep install --from-paths src --ignore-src -y \
#&& sudo apt-get autoremove && sudo apt-get autoclean \
#&& echo "-> Tools installed!"

# These were moved here because they only need to be run once!
echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> $HOME/.bashrc
echo "source '$HOME/ros_ws/install/local_setup.bash'" >> $HOME/.bashrc

echo "-> Initializing ROSDEP..."
#sudo rosdep init
rosdep update

echo "-> Installing package dependencies..."
cd "$HOME/ros_ws" || exit 1
NON_ROSDEP_DEPS="libxcb-cursor0 python3-lgpio python3-gpiozero"

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select ros_robot_msgs remote_pico_coms
source "./install/local_setup.sh"

export PIP_BREAK_SYSTEM_PACKAGES=1
rosdep install --from-paths src -y --ignore-src
sudo apt-get update && sudo apt-get install $NON_ROSDEP_DEPS -y
colcon build --packages-skip ros_robot_msgs remote_pico_coms

echo "--> post_create.sh done!"