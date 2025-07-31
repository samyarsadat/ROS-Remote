#!/usr/bin/env bash
# Devcontainer post-start script.
# This script runs every time the devcontainer is started.

set -e
echo "--> post_start.sh started!"
echo "-> Setting up environment..."

echo "-> Sourcing ROS..."
source /opt/ros/"$ROS_DISTRO"/setup.bash

echo "--> post_start.sh done!"