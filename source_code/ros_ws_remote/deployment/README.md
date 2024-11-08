These scripts are for setting up, running, and updating the ROS Remote GUI. The GUI should
be run on a Raspberry Pi (4B 8GB in my case, but a model with less RAM will also work) 
running Ubuntu 22.04 LTS. Raspberry Pi OS will not work.<br>
<br>
You should first run the setup script, then you can add the `run_gui.bash` script to start up
the list of startup programs using the GNOME "Startup Applications" app. Be sure to specify
the `-r` flag for the `run_gui.bash` script. This makes sure that the screen resolution is set properly.
You can enter this into the "Command" field:
```
/bin/bash "/home/[YOUR USERNAME]/ros_remote/source_code/ros_ws_remote/deployment/run_gui.bash" -r
```
Replace "[YOUR USERNAME]" with your username.<br>
<br>

## Note on Wayland
These scripts are written with the assumption that the X Window System is being used. If you
are using Wayland, you must either modify these scripts yourself, or you must switch to X11.<br>
<br>
On Ubuntu, you can switch to X11 by logging out, clicking on the settings icon, selecting 
"Ubuntu on Xorg", and then entering your passcode and logging in normally. You only need to
do this once, as the change is persistent.