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
cd $HOME/ros_ws && colcon build
source $HOME/ros_ws/install/local_setup.bash
cd $HOME/ros_ws

exec "$@"