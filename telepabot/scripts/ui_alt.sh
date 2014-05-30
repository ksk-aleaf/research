#!/bin/sh
export ROS_IP=172.24.40.84
export ROS_MASTER_URI=http://172.24.40.107:11311/
gnome-terminal -e 'bash -c "rosrun image_transport republish compressed in:=camera/rgb/image_color raw out:=camera/rgb/image_color_decompressed"'
roslaunch ui_alt ui_alt_on.launch
