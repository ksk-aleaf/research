#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('ui_alt')
import rospy
import copy

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
from sensor_msgs.msg import Joy

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import cv
import threading
import time
import math
import tf
import os

#グローバル変数モジュール
import global_var

#定数モジュール
import const

#音源の角度を引数にとり、付与するIDを返す
'''def getSourceDirectionId(theta):
	
	if(theta >= -10 and theta < 10):
		return 0
	elif(theta >= -30 and theta < -10):
		return 1
	elif(theta >= 10 and theta <= 30):
		return 2
	elif(theta > -75 and theta < -30):
		return 3
	elif(theta > -105 and theta <= -75):
		return 4
	elif(theta > -135 and theta <= -105):
		return 5
	elif(theta > -165 and theta <= -135):
		return 6
	elif(theta > 30 and theta <= 75):
		return 7
	elif(theta > 75 and theta <= 105):
		return 8
	elif(theta > 105 and theta <= 135):
		return 9
	elif(theta > 135 and theta <= 165):
		return 10
	elif((theta > 165 and theta <= 180) or (theta >= -180 and theta <= -165)):
		return 11
'''



#マニュアル操作ボタンのable/disable切り替え
def switchManBtnAble(flag):
	global_var.leftRotateButton.setEnabled(flag)
	global_var.rightRotateButton.setEnabled(flag)
	global_var.forwardButton.setEnabled(flag)
	global_var.backButton.setEnabled(flag)



#分離音声視聴リセットボタン
class AllListenButton(QtGui.QPushButton):
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.clicked.connect(self.resetSeparateListen)

	def resetSeparateListen(self):
		trigger = rospy.Publisher('Trigger', Bool)
		trigger.publish(False)



#長押し対応ボタン
class ManualRotateButton(QtGui.QPushButton):
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.setAutoRepeat(True)
		self.setAutoRepeatDelay(0)
		self.clicked.connect(self.sendManualCommand)
		self._state = 0


	def setButtonCode(self,code):#ボタンの種類(引数)毎に送るコマンドを設定
		self.pub = rospy.Publisher('/cmd_vel', Twist)
		self.command = Twist()
		if code == const.R_ROT_BTN_CODE:
			self.command = const.R_ROT_CMD
		elif code == const.L_ROT_BTN_CODE:
			self.command = const.L_ROT_CMD
		elif code == const.FWD_BTN_CODE:
			self.command = const.FWD_CMD
		elif code == const.BACK_BTN_CODE:
			self.command = const.BACK_CMD

	def sendManualCommand(self):#turtlebotに回転・移動コマンドを送信
		if self.isDown():#押している間コマンドを送り続ける
			sendCommand(self.command,const.MAN_ROT_TO)
			print 'repeat'
		elif self._state == 1:
			self._state = 0
			os.system(const.SET_TO_STR + str(const.DEFAULT_TO))
			print 'release'
		else:
			print 'click'


#オート回転ボタン
class AutoRotateButton(QtGui.QPushButton):
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.clicked.connect(self.sendAutoCommand)
		self.timeout = 0
		self.command = Twist()
		self.setGeometry(0,0,const.AUTO_ROT_BTN_WID,const.AUTO_ROT_BTN_HT)
		self.setIconSize(QtCore.QSize(const.AUTO_ROT_BTN_WID,const.AUTO_ROT_BTN_HT))
		self.setVisible(False)

	def sendAutoCommand(self):#turtlebotに回転・移動コマンドを送信
		global_var.manualButtonAbleFlag  = False
		global_var.manualFinDisableTime = time.time() + self.timeout + const.SEND_CMD_OVERHEAD
		button.switchManBtnAble(False)
		sendCommand(self.command,self.timeout)


#分離音声視聴ボタン
class ListenButton(QtGui.QPushButton):
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.clicked.connect(self.listen)
		self.theta = 0
		self.setGeometry(0,0,const.LIS_BTN_WID,const.LIS_BTN_HT)
		self.setIconSize(QtCore.QSize(const.LIS_BTN_WID,const.LIS_BTN_HT))
		self.setVisible(False)
	
	def listen(self):
		global_var.msg_select = HarkSource()
		global_var.msg_select.exist_src_num = 1
		trigger = rospy.Publisher('Trigger', Bool)
		pub_select = rospy.Publisher('SourceSelect', HarkSource)

		append_msg = HarkSourceVal()
		append_msg.id = getSourceDirectionId(self.theta)
		append_msg.theta = self.theta
		global_var.msg_select.src.append(append_msg)

		pub_select.publish(global_var.msg_select)
		r = rospy.Rate(3)
		r.sleep()
		trigger.publish(True)
