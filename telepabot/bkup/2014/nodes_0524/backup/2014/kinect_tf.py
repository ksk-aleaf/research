#! /usr/bin/env python
# -*-coding: utf-8 -*-
import rospy
import roslib; roslib.load_manifest('ui_alt')
from ui_alt.msg import tf_uialt
import global_var

#kinectのトラッキングデータをグローバル配列に格納
def tf_callback1(data):
	
	global_var.k1 = []

	for i in range(len(data.src)):
		global_var.k1.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	print "k1"
	print global_var.k1
	
def tf_callback2(data):

	
	global_var.k2 = []

	for i in range(len(data.src)):
		k2.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	
def tf_callback3(data):

	global_var.k3 = []

	for i in range(len(data.src)):
		global_var.k3.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	
def tf_callback4(data):

	global_var.k4 = []

	for i in range(len(data.src)):
		global_var.k4.append((data.src[i].x, data.src[i].y, data.src[i].theta))


#トピック購読処理
def subscriber():
	rospy.Subscriber('tf_processed1', tf_uialt, tf_callback1)
	rospy.Subscriber('tf_processed2', tf_uialt, tf_callback2)
	rospy.Subscriber('tf_processed3', tf_uialt, tf_callback3)
	rospy.Subscriber('tf_processed4', tf_uialt, tf_callback4)
