#!/bin/bash
ROS_DISTRO="jazzy"
set -e

while getopts rgb flag
do
    case "${flag}" in
        b) FORCE_REBUILD="true";;
        g) REBUILD_GUI_ONLY="true";;
        r) FORCE_RESET="true";;
        *) echo "Invalid flags! (-r: reset repository, -b: run colcon build regardless of up-to-dateness, -g: same as -b but only performs build for the GUI packages)" && exit 1;;
    esac
done

cd "$HOME/ros_remote" || exit 1
git fetch origin
GIT_BRANCH=$(git branch --show-current)
IS_UPTODATE=$(git diff "origin/$GIT_BRANCH")

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_RESET" == "true" ]; then
    git clean -dfx
    git reset --recurse-submodules --hard
    git pull origin "$GIT_BRANCH"
    git submodule update --recursive
fi

if [ "$IS_UPTODATE" != "" ] || [ "$FORCE_REBUILD" == "true" ]; then
    SOURCE_CODE_PATH="$HOME/ros_remote/source_code"

    source /opt/ros/$ROS_DISTRO/setup.bash
    cd "$SOURCE_CODE_PATH/ros_ws_remote" || exit 1
    colcon build --packages-skip test_camera_publisher

    cd "$SOURCE_CODE_PATH/ros_ws_remote/src/ros_remote_gui/util_scripts" || exit 1
    bash ./generate_ui_py_files.sh

    if [ "$REBUILD_GUI_ONLY" == "" ]; then
        cd "$SOURCE_CODE_PATH/ros_robot_project/source_code/ros_ws_robot_infra" || exit 1
        colcon build --packages-select ros_robot_msgs
    fi
fi

echo "All up to date with $GIT_BRANCH."