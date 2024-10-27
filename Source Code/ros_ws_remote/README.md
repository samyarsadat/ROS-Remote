## Note on Wayland
The Docker configuration for the remote GUI assumes that X11 is being used.
The following steps regarding display resolution also make that assumption.
It is recommended that you switch to X11 using `raspi-config`, or else you will need to adjust
the Docker configuration and find a Wayland-specific way to force the display resolution.
<br><br>

## Raspberry Pi Display Resolution
The native resolution of the 7-inch Waveshare display used in this project is 1024x600 pixels.<br>
Because 1024x600 is not a standard resolution, it cannot be selected from the list of available in the
display resolution utility available on Raspberry Pi OS.<br>
<br>
To prevent resolution scaling, we need to manually set the HDMI output resolution to 1024x600.<br>
To do this, you need to run a few commands.<br>
<br>
Firstly, install CVT by running `sudo apt-get install xcvt`, then run `cvt 1024 600 60`.<br>
The command will return something like this:
```
# 1024x600 59.85 Hz (CVT) hsync: 37.35 kHz; pclk: 49.00 MHz
Modeline "1024x600_60.00"   49.00  1024 1072 1168 1312  600 603 613 624 -hsync +vsync
```
Take the second line of the output (excluding "Modeline") and paste it in place of the one
on line 27 of the `entrypoint.sh` inside the `.prodcontainer` directory.
Make sure to properly save the file.