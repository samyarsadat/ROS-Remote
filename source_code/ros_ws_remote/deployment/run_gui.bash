#!/bin/bash
set -e

DISPLAY_HDMI_PORT="HDMI-1"
DISPLAY_RES_HEIGHT=600
DISPLAY_RES_WIDTH=1024
DISPLAY_FPS=60

while getopts wr flag
do
    case "${flag}" in
        w) sleep 10;;
        r) SET_SCRN_RES="true";;
        *) echo "Invalid flags! (-w: wait 10 seconds before start, -r: set screen resolution)" && exit 1;;
    esac
done

if [ "$SET_SCRN_RES" == "true" ]; then
    if ! DISPLAY=:0 xrandr | sed -n "/^"$DISPLAY_HDMI_PORT"/,/^[^ ]/p" | grep -q $DISPLAY_RES_WIDTH"x"$DISPLAY_RES_HEIGHT"_"$DISPLAY_FPS".00"; then
        CVT_MODELINE_OUT=$(cvt $DISPLAY_RES_WIDTH $DISPLAY_RES_HEIGHT $DISPLAY_FPS | grep "Modeline" | sed 's/Modeline "\(.*\)"/\1/')
        DISPLAY=:0 xrandr --newmode $CVT_MODELINE_OUT
        DISPLAY=:0 xrandr --addmode $DISPLAY_HDMI_PORT $DISPLAY_RES_WIDTH"x"$DISPLAY_RES_HEIGHT"_"$DISPLAY_FPS".00"
    fi

    DISPLAY=:0 xrandr --output $DISPLAY_HDMI_PORT --mode $DISPLAY_RES_WIDTH"x"$DISPLAY_RES_HEIGHT"_"$DISPLAY_FPS".00"
fi

cd "$HOME/ros_remote/source_code/ros_ws_remote" || exit 1
#source /opt/ros/humble/setup.bash
source /opt/vulcanexus/humble/setup.bash
source install/local_setup.sh
source "../pico_ws/libmicroros/install/local_setup.sh"
source "../ros_robot_project/source_code/ros_ws_robot/install/local_setup.sh"
DISPLAY=:0 ros2 launch ros_remote_gui gui_launch.py
#DISPLAY=:0 ros2 run ros_remote_gui remote_gui_node