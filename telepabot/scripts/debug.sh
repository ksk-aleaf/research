#!/bin/sh
gnome-terminal -e 'bash -c "rosrun image_transport republish compressed in:=camera/rgb/image_color raw out:=camera/rgb/image_color_decompressed"'
roslaunch ui_alt debug.launch
