#!/usr/bin/env bash
# Devcontainer post-create script.
# This script is run after the devcontainer is created.

set -e
echo "--> post_create.sh started!"
echo "-> First container run, running MicroROS tools setup..."

echo "-> Setting folder permissions and copying files..."
sudo chown -R nonroot: $HOME/pico_ws/
cp "$HOME/pico_ws/libfreertos/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake" \
    $HOME/pico_ws/FreeRTOS_Kernel_import.cmake

echo "export PATH='$PATH:$HOME/.local/bin'" >> $HOME/.bashrc
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $HOME/.bashrc

echo "-> Installing ELF size analyzer..."
pip install --no-warn-script-location --break-system-packages elf-size-analyze

echo "--> post_create.sh done!"