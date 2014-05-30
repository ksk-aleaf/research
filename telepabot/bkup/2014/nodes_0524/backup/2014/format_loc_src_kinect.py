#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('ui_alt')
import rospy

#必要なメッセージファイル
from hark_msgs.msg import HarkSource
from hark_msgs.msg import HarkSourceVal
from hark_msgs.msg import HarkSrcWave
from hark_msgs.msg import HarkSrcWaveVal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as SIm
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from ui_alt.msg import tf_uialt

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
import cv
import threading
import time
import math
import tf
import os

#定数
CAMERA_ANGLE_THRESHOLD = 28.5

#グローバル変数
max_power = 0
source = []
prev_msg = HarkSource()


#Show localization information
def publish_localization(data):

	global source
	global max_power
	global prev_msg

	source = []

	pub_formatted = rospy.Publisher('FormattedSource', HarkSource)
	pub_detected = rospy.Publisher('DetectedSource', HarkSource)
	detected_harksrc = HarkSource()
	msg_formatted = HarkSource()
	
	for i in range(len(sub_harksrc.src)):
	
		if(sub_harksrc.src[i].theta >= -45 and sub_harksrc.src[i].theta <= 45):#音源角度から、kinectの画像上のx座標を求める
			x = 320*(1-(math.tan(math.radians(sub_harksrc.src[i].theta))/math.tan(math.radians(28.5))))
		else:#画面外から音が聞こえた場合と思われる
			x = -100

		array = [sub_harksrc.src[i].id, x, sub_harksrc.src[i].theta, sub_harksrc.src[i].power]
		msg_formatted.src.append(array)

		max_power = msg_formatted.src[0][3]
		for j in range(len(msg_formatted.src)):
			if msg_formatted.src[j][3] - max_power > 0:
				max_power = msg_formatted.src[j][3]

	#detectedSourceにデータを入れる
	for k in range(len(sub_harksrc.src)):
		tmp_val = HarkSourceVal()
		if(sub_harksrc.src[k].theta >= -10 and sub_harksrc.src[k].theta < 10):
			tmp_val.id = 0
		elif(sub_harksrc.src[k].theta >= -30 and sub_harksrc.src[k].theta < -10):
			tmp_val.id = 1
		elif(sub_harksrc.src[k].theta >= 10 and sub_harksrc.src[k].theta <= 30):
			tmp_val.id = 2
		elif(sub_harksrc.src[k].theta > -75 and sub_harksrc.src[k].theta < -30):
			tmp_val.id = 3
		elif(sub_harksrc.src[k].theta > -105 and sub_harksrc.src[k].theta <= -75):
			tmp_val.id = 4
		elif(sub_harksrc.src[k].theta > -135 and sub_harksrc.src[k].theta <= -105):
			tmp_val.id = 5
		elif(sub_harksrc.src[k].theta > -165 and sub_harksrc.src[k].theta <= -135):
			tmp_val.id = 6
		elif(sub_harksrc.src[k].theta > 30 and sub_harksrc.src[k].theta <= 75):
			tmp_val.id = 7
		elif(sub_harksrc.src[k].theta > 75 and sub_harksrc.src[k].theta <= 105):
			tmp_val.id = 8
		elif(sub_harksrc.src[k].theta > 105 and sub_harksrc.src[k].theta <= 135):
			tmp_val.id = 9
		elif(sub_harksrc.src[k].theta > 135 and sub_harksrc.src[k].theta <= 165):
			tmp_val.id = 10
		elif((sub_harksrc.src[k].theta > 165 and sub_harksrc.src[k].theta <= 180) or (sub_harksrc.src[k].theta >= -180 and sub_harksrc.src[k].theta <= -165)):
			tmp_val.id = 11
		tmp_val.x  = sub_harksrc.src[k].x
		tmp_val.y  = sub_harksrc.src[k].y
		tmp_val.theta = int((sub_harksrc.src[k].theta - CAMERA_ANGLE_THRESHOLD)/5)*20
		detected_harksrc.src.append(tmp_val)
		detected_harksrc.exist_src_num += 1

	#直前のデータと比較して、直前のデータに含まれないものがあればソース数を追加
	for l in range(len(prev_msg.src)):
		flag = False
		for n in range(len(detected_harksrc.src)):
			if(prev_msg.src[l].id == detected_harksrc.src[n].id):
				flag = True
		if(flag == False):
			detected_harksrc.src.append(prev_msg.src[l])
			detected_harksrc.exist_src_num += 1

	#detectedSourceを発行してデータの控えを取得
	pub_detected.publish(detected_harksrc)
	prev_msg = detected_harksrc


def formatLocSrc():

	#rospy.init_node('UIALT', anonymous = True)
	rospy.Subscriber('HarkSource', HarkSource, publish_localization, buff_size = 1)
	rospy.spin()


if __name__ == '__main__':
	formatLocSrc()
