#!/bin/sh
rosrun image_transport republish compressed in:=camera/rgb/image_color raw out:=camera/rgb/image_color_decompressed
