#!/bin/bash
set -e

while getopts r flag
do
    case "${flag}" in
        r) FORCE_REBUILD="true";;
        *) echo "Invalid flags! (-r: run colcon build regardless of up-to-dateness)" && exit 1;;
    esac
done

cd "$HOME/ros_remote" || exit 1
git fetch origin
IS_UPTODATE=$(git diff origin/main)

if [ "$IS_UPTODATE" != "" ]; then
    git clean -df
    git reset --hard
    git pull origin main
    git submodule update --recursive
fi

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_REBUILD" == "true" ]; then
    source /opt/ros/humble/setup.bash
    cd "./Source Code/ros_ws_remote" || exit 1
    colcon build --packages-skip test_camera_publisher
    cd "../pico_ws/libmicroros" || exit 1
    colcon build --packages-select remote_pico_coms
    cd "../../ros_robot_project/Source Code/ros_ws_robot" || exit 1
    colcon build --packages-select ros_robot_msgs
fi

echo "All up to date."