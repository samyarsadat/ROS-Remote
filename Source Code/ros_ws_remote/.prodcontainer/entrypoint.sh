#!/usr/bin/env bash

set -e
echo "-> entrypoint.sh"

if ! test -f /not_first_run; then
    echo "-> First container run, running setup..."

    echo "-> Setting folder permissions and copying files..."
    sudo chown -R nonroot: $HOME/ros_ws/

    echo "source '/opt/ros/$ROS_DISTRO/setup.bash'" >> $HOME/.bashrc
    echo "source '$HOME/ros_ws/install/local_setup.bash'" >> $HOME/.bashrc

    sudo touch /not_first_run
fi

echo "-> Sourcing ROS..."
source /opt/ros/"$ROS_DISTRO"/setup.bash

echo "-> Colcon build..."
cd $HOME/ros_ws && colcon build --packages-skip test_camera_publisher
source $HOME/ros_ws/install/local_setup.bash
cd $HOME/ros_ws

# Force display resolution to 1024x600!
xrandr --newmode "1024x600_60.00" 49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync
xrandr --addmode HDMI-1 "1024x600_60.00"
xrandr --output HDMI-1 --mode "1024x600_60.00"

exec "$@"