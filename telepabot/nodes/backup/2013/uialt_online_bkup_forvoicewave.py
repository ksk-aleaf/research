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
#ボタンモジュール
import button
from button import AllListenButton
from button import ManualRotateButton
from button import AutoRotateButton
from button import ListenButton
#kinect tracking file モジュール
import kinect_tf
#定位データフォーマットモジュール
import format_loc_src_microcone

FRAMES = [
	'head',
	'neck',
	'torso',
	'left_shoulder',
	'right_shoulder'
	]












#画面内の音を選択する用
def src_select():
	global_var.msg_select = HarkSource()
	global_var.msg_select.exist_src_num = 0
	trigger = rospy.Publisher('Trigger', Bool)
	pub_select = rospy.Publisher('SourceSelect', HarkSource)
	select = False
	flag = False

	#何も選択されていない場合，生データ再生
	if(global_var.sflag == False):

		global_var.msg_select.src = []
		trigger.publish(select)
		global_var.msg_select.exist_src_num = 0

	#選択された場合
	if(global_var.sflag == True and global_var.flag == False):

		if(global_var.cflag == True):

			for i in range(len(global_var.max_theta)):
				append = HarkSourceVal()
				if((global_var.min_theta[i] + global_var.max_theta[i]) / 2 >= -10 and (global_var.min_theta[i] + global_var.max_theta[i]) / 2 < 10):
					append.id = 0
				if((global_var.min_theta[i] + global_var.max_theta[i]) / 2 >= -30 and (global_var.min_theta[i] + global_var.max_theta[i]) / 2 < -10):
					append.id = 2
				if((global_var.min_theta[i] + global_var.max_theta[i]) / 2 >= 10 and (global_var.min_theta[i] + global_var.max_theta[i]) / 2 < 30):
					append.id = 1

			global_var.msg_select.src.append(append)
			flag = True
			global_var.msg_select.exist_src_num = len(global_var.max_theta)
			select = True

	if(global_var.dflag == True):
		for i in range(len(global_var.d_theta)):
			append = HarkSourceVal()
			if(global_var.d_theta[i] >= -10 and global_var.d_theta[i] < 10):
				append.id = 0
			if(global_var.d_theta[i] >= -30 and global_var.d_theta[i] < -10):
				append.id = 1
			if(global_var.d_theta[i] >= 10 and global_var.d_theta[i] < 30):
				append.id = 2
			global_var.msg_select.src.append(append)
			global_var.flag = True
			global_var.msg_select.exist_src_num = len(global_var.d_theta)
			select = True
	
	pub_select.publish(global_var.msg_select)
	r = rospy.Rate(3)
	r.sleep()
	trigger.publish(select)

#turtlebotへコマンドを送信
def sendCommand(command,timeout):
	os.system(const.SET_TO_STR + str(timeout))
	pub = rospy.Publisher('/cmd_vel', Twist)
	pub.publish(command)
	rospy.loginfo(command)






#OpenCVIplImageからPyQtのQImageへの変換クラス http://rafaelbarreto.wordpress.com/tag/pyqt/
class OpenCVQImage(QtGui.QImage):

	def __init__(self, opencvBgrImg):
		depth, nChannels = opencvBgrImg.depth, opencvBgrImg.nChannels
		if depth != cv.IPL_DEPTH_8U or nChannels != 3:
			raise ValueError("the input image must be 8-bit, 3-channel")
		w, h = cv.GetSize(opencvBgrImg)
		opencvRgbImg = cv.CreateImage((w, h), depth, nChannels)

		# it's assumed the image is in BGR format
		cv.CvtColor(opencvBgrImg, opencvRgbImg, cv.CV_BGR2RGB)
		self._imgData = opencvRgbImg.tostring()
		super(OpenCVQImage, self).__init__(self._imgData, w, h, QtGui.QImage.Format_RGB888)



class CameraDevice(QtCore.QObject):

	global_var.cv_image

	_DEFAULT_FPS = 30

	newFrame = QtCore.pyqtSignal(cv.iplimage)

	def __init__(self, mirrored=False, parent=None):

		super(CameraDevice, self).__init__(parent)

		self.mirrored = mirrored
		self._timer = QtCore.QTimer(self)
		self._timer.timeout.connect(self._queryFrame)
		self._timer.setInterval(1000/self.fps)
		self.paused = False
		self.bridge = CvBridge()
		self.im_sub = rospy.Subscriber("/camera/rgb/image_color_decompressed", SIm, self.image_callback)
		#self.im_sub=rospy.Subscriber("/camera/rgb/image_color",SIm, self.image_callback)

	def image_callback(self, data):
		global_var.cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")

	@property
	def paused(self):
		return not self._timer.isActive()

	@paused.setter
	def paused(self, p):
		if p:
			self._timer.stop()
		else:
			self._timer.start()

	@property
	def frameSize(self):
		w = const.WIN_WID
		h = const.WIN_HT
		return int(w), int(h)

	@property
	def fps(self):
		fps = self._DEFAULT_FPS
		return fps
	
	@QtCore.pyqtSlot()
	def _queryFrame(self):
		#frame = cv.QueryFrame(self._cameraDevice)
		frame = global_var.cv_image

		if frame == None:
			return

		if self.mirrored:
			mirroredFrame = cv.CreateImage(cv.GetSize(frame), 8, 3)
			cv.Copy(frame, mirroredFrame, None)
			#cv.Flip(frame, mirroredFrame, 1)
			frame =  mirroredFrame
		self.newFrame.emit(frame)




#カメラウィジェット作成クラス.
class CameraWidget(QtGui.QWidget):
	newFrame = QtCore.pyqtSignal(cv.iplimage)

	def __init__(self, cameraDevice, parent=None):

		super(CameraWidget, self).__init__(parent)
		self._frame = None
		self.frame = QtGui.QFrame(self)
		self.frame.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Raised)
		self.frame.setLineWidth(2)
		self.frame.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)


		self._cameraDevice = cameraDevice
		self._cameraDevice.newFrame.connect(self._onNewFrame)
		#w, h = self._cameraDevice.frameSize
		#self.setStyleSheet("background-color: white")#背景色を白に
		self.setMinimumSize(const.WIN_WID, const.WIN_HT)
		self.setMaximumSize(const.WIN_WID, const.WIN_HT)
#		self.text1 = 'Select'
		self.text2 = u'画面外から話し声がします'

		#視聴範囲
		self.listenRangeStartX = 0
		self.listenRangeEndX = 0

		#ボタンアイコン
		self.auto_left_rotate_button_icon = QtGui.QIcon(self)
		self.auto_left_rotate_button_icon.addFile("/home/user/ros/ui_alt/icon/character_left.png")
		self.auto_right_rotate_button_icon = QtGui.QIcon(self)
		self.auto_right_rotate_button_icon.addFile("/home/user/ros/ui_alt/icon/character_right.png")

		#右矢印1
		self.arrow_right_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x1.png")
		self.arrow_right_x1_image = self.arrow_right_x1_image.scaled(const.ARW_WID_1,const.ARW_HT,0,0)

		#右矢印2
		self.arrow_right_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x2.png")
		self.arrow_right_x2_image = self.arrow_right_x2_image.scaled(const.ARW_WID_2,const.ARW_HT,0,0)

		#右矢印3
		self.arrow_right_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x3.png")
		self.arrow_right_x3_image = self.arrow_right_x3_image.scaled(const.ARW_WID_3,const.ARW_HT,0,0)

		#右矢印1(薄い)
		self.arrow_right_x1_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x1_pale.png")
		self.arrow_right_x1_pale_image = self.arrow_right_x1_pale_image.scaled(const.ARW_WID_1,const.ARW_HT,0,0)

		#右矢印2(薄い)
		self.arrow_right_x2_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x2_pale.png")
		self.arrow_right_x2_pale_image = self.arrow_right_x2_pale_image.scaled(const.ARW_WID_2,const.ARW_HT,0,0)

		#右矢印3(薄い)
		self.arrow_right_x3_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x3_pale.png")
		self.arrow_right_x3_pale_image = self.arrow_right_x3_pale_image.scaled(const.ARW_WID_3,const.ARW_HT,0,0)


		#左矢印1
		self.arrow_left_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x1.png")
		self.arrow_left_x1_image = self.arrow_left_x1_image.scaled(const.ARW_WID_1,const.ARW_HT,0,0)

		#左矢印2
		self.arrow_left_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x2.png")
		self.arrow_left_x2_image = self.arrow_left_x2_image.scaled(const.ARW_WID_2,const.ARW_HT,0,0)

		#左矢印3
		self.arrow_left_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x3.png")
		self.arrow_left_x3_image = self.arrow_left_x3_image.scaled(const.ARW_WID_3,const.ARW_HT,0,0)

		#左矢印1(薄い)
		self.arrow_left_x1_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x1_pale.png")
		self.arrow_left_x1_pale_image = self.arrow_left_x1_pale_image.scaled(const.ARW_WID_1,const.ARW_HT,0,0)

		#左矢印2(薄い)
		self.arrow_left_x2_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x2_pale.png")
		self.arrow_left_x2_pale_image = self.arrow_left_x2_pale_image.scaled(const.ARW_WID_2,const.ARW_HT,0,0)

		#左矢印3(薄い)
		self.arrow_left_x3_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x3_pale.png")
		self.arrow_left_x3_pale_image = self.arrow_left_x3_pale_image.scaled(const.ARW_WID_3,const.ARW_HT,0,0)

		#右向きキャラ
		self.character_right_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_right.png")
		self.character_right_image = self.character_right_image.scaled(const.ONIMG_LIS_BTN_WID, const.ONIMG_LIS_BTN_HT,0,0)


		#左向きキャラ
		self.character_left_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_left.png")
		self.character_left_image = self.character_left_image.scaled(const.ONIMG_LIS_BTN_WID, const.ONIMG_LIS_BTN_HT,0,0)


		#音声視聴ボタン
		self.left_listen_button_icon = QtGui.QIcon(self)
		self.left_listen_button_icon.addFile("/home/user/ros/ui_alt/icon/character_left.png")
		self.right_listen_button_icon = QtGui.QIcon(self)
		self.right_listen_button_icon.addFile("/home/user/ros/ui_alt/icon/character_right.png")

		#左回転ボタン
		self.left_rotate_icon = QtGui.QIcon(self)
		self.left_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		global_var.leftRotateButton = ManualRotateButton(self.left_rotate_icon,"",self)
		global_var.leftRotateButton.setButtonCode(const.L_ROT_BTN_CODE)
		global_var.leftRotateButton.setGeometry(const.MAN_L_ROT_BTN_X,const.MAN_L_ROT_BTN_Y,const.MAN_L_ROT_BTN_WID,const.MAN_L_ROT_BTN_HT)
		global_var.leftRotateButton.setIconSize(QtCore.QSize(const.MAN_L_ROT_BTN_WID,const.MAN_L_ROT_BTN_HT))


		#右回転ボタン
		self.right_rotate_icon = QtGui.QIcon(self)
		self.right_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		global_var.rightRotateButton
		global_var.rightRotateButton = ManualRotateButton(self.right_rotate_icon,"",self)
		global_var.rightRotateButton.setButtonCode(const.R_ROT_BTN_CODE)
		global_var.rightRotateButton.setGeometry(const.MAN_R_ROT_BTN_X,const.MAN_R_ROT_BTN_Y,const.MAN_R_ROT_BTN_WID,const.MAN_R_ROT_BTN_HT)
		global_var.rightRotateButton.setIconSize(QtCore.QSize(const.MAN_R_ROT_BTN_WID,const.MAN_R_ROT_BTN_HT))


		#前進ボタン
		self.forward_icon = QtGui.QIcon(self)
		self.forward_icon.addFile("/home/user/ros/ui_alt/icon/arrow_forward.png")
		global_var.forwardButton = ManualRotateButton(self.forward_icon,"",self)
		global_var.forwardButton.setButtonCode(const.FWD_BTN_CODE)
		global_var.forwardButton.setGeometry(const.MAN_FWD_X,const.MAN_FWD_Y,const.MAN_FWD_WID,const.MAN_FWD_HT)
		global_var.forwardButton.setIconSize(QtCore.QSize(const.MAN_FWD_WID,const.MAN_FWD_HT))

		#後退ボタン
		self.back_icon = QtGui.QIcon(self)
		self.back_icon.addFile("/home/user/ros/ui_alt/icon/arrow_back.png")
		global_var.backButton = ManualRotateButton(self.back_icon,"",self)
		global_var.backButton.setButtonCode(const.BACK_BTN_CODE)
		global_var.backButton.setGeometry(const.MAN_BACK_X,const.MAN_BACK_Y,const.MAN_BACK_WID,const.MAN_BACK_HT)
		global_var.backButton.setIconSize(QtCore.QSize(const.MAN_BACK_WID,const.MAN_BACK_HT))
		
		#分離音声視聴リセットボタン
		self.all_listen_icon = QtGui.QIcon(self)
		self.all_listen_icon.addFile("/home/user/ros/ui_alt/icon/listen.png")
		global_var.allListenButton = AllListenButton(self.all_listen_icon,"",self)
		global_var.allListenButton.setGeometry(const.ALL_LIS_BTN_X,const.ALL_LIS_BTN_Y,const.ALL_LIS_BTN_WID,const.ALL_LIS_BTN_HT)
		global_var.allListenButton.setIconSize(QtCore.QSize(const.ALL_LIS_BTN_WID,const.ALL_LIS_BTN_HT))

		#左回転ボタン(自動)
		for i in range(const.AUTO_ROT_BTN_NUM):
			button = AutoRotateButton(self.left_rotate_icon,"",self)
			button.command = const.L_ROT_CMD
			global_var.leftAutoRotBtnList.append(button)

		#右回転ボタン(自動)
		for i in range(const.AUTO_ROT_BTN_NUM):
			button = AutoRotateButton(self.right_rotate_icon,"",self)
			button.command = const.R_ROT_CMD
			global_var.rightAutoRotBtnList.append(button)

		#左分離音声視聴ボタン
		for i in range(const.LIS_BTN_NUM):
			button = ListenButton(self.left_listen_button_icon,"",self)
			global_var.leftListenBtnList.append(button)

		#右分離音声視聴ボタン
		for i in range(const.LIS_BTN_NUM):
			button = ListenButton(self.right_listen_button_icon,"",self)
			global_var.rightListenBtnList.append(button)

		#[画面内]左分離音声視聴ボタン
		for i in range(const.LIS_BTN_NUM/2):
			button = ListenButton(self.left_listen_button_icon,"",self)
			global_var.onImageLeftListenBtnList.append(button)

		#[画面内]右分離音声視聴ボタン
		for i in range(const.LIS_BTN_NUM/2):
			button = ListenButton(self.right_listen_button_icon,"",self)
			global_var.onImageRightListenBtnList.append(button)



	@QtCore.pyqtSlot(cv.iplimage)
	def _onNewFrame(self, frame):
		self._frame = cv.CloneImage(frame)
		self.newFrame.emit(self._frame)
		self.update()

	def changeEvent(self, e):
		if e.type() == QtCore.QEvent.EnabledChange:
			if self.isEnabled():
				self._cameraDevice.newFrame.connect(self._onNewFrame)
			else:
				self._cameraDevice.newFrame.disconnect(self._onNewFrame)




	def paintVoiceWave

	#人の口の近くに音の波を描画
	def paintVoiceWave(self,e):
		painter2 = QtGui.QPainter(self)#直接選択時の「SELECT」用
		painter3 = QtGui.QPainter(self)
		painter4 = QtGui.QPainter(self)#定位情報表示用
		painter5 = QtGui.QPainter(self)
		
		painter3.setRenderHint(QtGui.QPainter.Antialiasing, True)#描画時のエンジン指定?
		painter4.setRenderHint(QtGui.QPainter.Antialiasing, True)
		pen1 = QtGui.QPen(QtGui.QColor(255, 0, 255), 5, QtCore.Qt.SolidLine)
		pen2 = QtGui.QPen(QtCore.Qt.green, 1, QtCore.Qt.SolidLine)
		pen3 = QtGui.QPen(QtCore.Qt.yellow, 2, QtCore.Qt.SolidLine)
		pen4 = QtGui.QPen(QtCore.Qt.red, 3, QtCore.Qt.SolidLine)
		pen5 = QtGui.QPen(QtCore.Qt.white, 3, QtCore.Qt.SolidLine)
		
		path1_1 = QtGui.QPainterPath()
		path1_2 = QtGui.QPainterPath()
		path1_3 = QtGui.QPainterPath()
		path1_4 = QtGui.QPainterPath()
		path1_5 = QtGui.QPainterPath()
		path2_1 = QtGui.QPainterPath()
		path2_2 = QtGui.QPainterPath()
		path2_3 = QtGui.QPainterPath()
		path2_4 = QtGui.QPainterPath()
		path2_5 = QtGui.QPainterPath()
		path3_1 = QtGui.QPainterPath()
		path3_2 = QtGui.QPainterPath()
		path3_3 = QtGui.QPainterPath()
		path3_4 = QtGui.QPainterPath()
		path3_5 = QtGui.QPainterPath()
		path4_1 = QtGui.QPainterPath()
		path4_2 = QtGui.QPainterPath()
		path4_3 = QtGui.QPainterPath()
		path4_4 = QtGui.QPainterPath()
		path4_5 = QtGui.QPainterPath()
		path5_1 = QtGui.QPainterPath()
		path5_2 = QtGui.QPainterPath()
		path5_3 = QtGui.QPainterPath()
		path5_4 = QtGui.QPainterPath()
		path5_5 = QtGui.QPainterPath()
		
		if(global_var.lflag == True):
			for i in range(len(global_var.locSrcList)):
				if(global_var.k1 == []):
					break
				print "not break"
				if(math.fabs(global_var.k1[0][2] - global_var.locSrcList[i].theta) <= 10.0):#音源の位置とkinectでトラッキングした人の位置が近い場合
					power1 = global_var.locSrcList[i].power
					path1_1.moveTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 10, global_var.k1[1][1] + 10)
					path1_2.moveTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 20, global_var.k1[1][1] + 20)
					path1_3.moveTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 30, global_var.k1[1][1] + 30)
					path1_4.moveTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 40, global_var.k1[1][1] + 40)
					path1_5.moveTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 50, global_var.k1[1][1] + 50)
					path1_1.cubicTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 10, global_var.k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 10, global_var.k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 10, global_var.k1[1][1] + 10)
					path1_2.cubicTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 20, global_var.k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 20, global_var.k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 20, global_var.k1[1][1] + 20)
					path1_3.cubicTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 30, global_var.k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 30, global_var.k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 30, global_var.k1[1][1] + 30)
					path1_4.cubicTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 40, global_var.k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 40, global_var.k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 40, global_var.k1[1][1] + 40)
					path1_5.cubicTo(global_var.k1[1][0] + const.CAM_IMG_OFS - 50, global_var.k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 50, global_var.k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k1[1][0] + const.CAM_IMG_OFS + 50, global_var.k1[1][1] + 50)
			

					if(global_var.max_power - power1 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)
						painter3.drawPath(path1_4)
						painter3.drawPath(path1_5)
			
					if(global_var.max_power - power1 > 0 and global_var.max_power - power1 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)
						painter3.drawPath(path1_4)
   
					if(global_var.max_power - power1 > 0.5 and global_var.max_power - power1 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)

					if(global_var.max_power - power1 > 1.0 and global_var.max_power - power1 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)

					if(global_var.max_power - power1 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path1_1)
			
			for i in range(len(global_var.locSrcList)):

				if(global_var.k2 == []):
					break
		
				if(math.fabs(global_var.k2[0][2] - global_var.locSrcList[i].theta) <= 10.0):
					power2 = global_var.locSrcList[i].power
					path2_1.moveTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 10, global_var.k2[1][1] + 10)
					path2_2.moveTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 20, global_var.k2[1][1] + 20)
					path2_3.moveTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 30, global_var.k2[1][1] + 30)
					path2_4.moveTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 40, global_var.k2[1][1] + 40)
					path2_5.moveTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 50, global_var.k2[1][1] + 50)
					path2_1.cubicTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 10, global_var.k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 10, global_var.k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 10, global_var.k2[1][1] + 10)
					path2_2.cubicTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 20, global_var.k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 20, global_var.k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 20, global_var.k2[1][1] + 20)
					path2_3.cubicTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 30, global_var.k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 30, global_var.k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 30, global_var.k2[1][1] + 30)
					path2_4.cubicTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 40, global_var.k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 40, global_var.k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 40, global_var.k2[1][1] + 40)
					path2_5.cubicTo(global_var.k2[1][0] + const.CAM_IMG_OFS - 50, global_var.k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 50, global_var.k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k2[1][0] + const.CAM_IMG_OFS + 50, global_var.k2[1][1] + 50)
			

					if(global_var.max_power - power2 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)
						painter3.drawPath(path2_4)
						painter3.drawPath(path2_5)
			
					if(global_var.max_power - power2 > 0 and global_var.max_power - power2 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)
						painter3.drawPath(path2_4)
	   
					if(global_var.max_power - power2 > 0.5 and global_var.max_power - power2 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)

					if(global_var.max_power - power2 > 1.0 and global_var.max_power - power2 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)

					if(global_var.max_power - power2 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path2_1)	 
			

			for i in range(len(global_var.locSrcList)):

				if(global_var.k3 == []):
					break

				if(math.fabs(global_var.k3[0][2] - global_var.locSrcList[i].theta) <= 10.0):
					power3 = global_var.locSrcList[i].power
					path3_1.moveTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 10, global_var.k3[1][1] + 10)
					path3_2.moveTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 20, global_var.k3[1][1] + 20)
					path3_3.moveTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 30, global_var.k3[1][1] + 30)
					path3_4.moveTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 40, global_var.k3[1][1] + 40)
					path3_5.moveTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 50, global_var.k3[1][1] + 50)
					path3_1.cubicTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 10, global_var.k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 10, global_var.k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 10, global_var.k3[1][1] + 10)
					path3_2.cubicTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 20, global_var.k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 20, global_var.k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 20, global_var.k3[1][1] + 20)
					path3_3.cubicTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 30, global_var.k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 30, global_var.k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 30, global_var.k3[1][1] + 30)
					path3_4.cubicTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 40, global_var.k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 40, global_var.k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 40, global_var.k3[1][1] + 40)
					path3_5.cubicTo(global_var.k3[1][0] + const.CAM_IMG_OFS - 50, global_var.k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 50, global_var.k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k3[1][0] + const.CAM_IMG_OFS + 50, global_var.k3[1][1] + 50)
			

					if(global_var.max_power - power3 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)
						painter3.drawPath(path3_4)
						painter3.drawPath(path3_5)
			
					if(global_var.max_power - power3 > 0 and global_var.max_power - power3 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)
						painter3.drawPath(path3_4)
	   
					if(global_var.max_power - power3 > 0.5 and global_var.max_power - power3 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)

					if(global_var.max_power - power3 > 1.0 and global_var.max_power - power3 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)

					if(global_var.max_power - power3 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path3_1)
			
			
			for i in range(len(global_var.locSrcList)):
			
				if(global_var.k4 == []):
					break

				if(math.fabs(global_var.k4[0][2] - global_var.locSrcList[i].theta) <= 10.0):
			
					power4 = global_var.locSrcList[i].power
					path4_1.moveTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 10, global_var.k4[1][1] + 10)
					path4_2.moveTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 20, global_var.k4[1][1] + 20)
					path4_3.moveTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 30, global_var.k4[1][1] + 30)
					path4_4.moveTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 40, global_var.k4[1][1] + 40)
					path4_5.moveTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 50, global_var.k4[1][1] + 50)
					path4_1.cubicTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 10, global_var.k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 10, global_var.k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 10, global_var.k4[1][1] + 10)
					path4_2.cubicTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 20, global_var.k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 20, global_var.k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 20, global_var.k4[1][1] + 20)
					path4_3.cubicTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 30, global_var.k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 30, global_var.k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 30, global_var.k4[1][1] + 30)
					path4_4.cubicTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 40, global_var.k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 40, global_var.k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 40, global_var.k4[1][1] + 40)
					path4_5.cubicTo(global_var.k4[1][0] + const.CAM_IMG_OFS - 50, global_var.k4[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 50, global_var.k4[1][1] + 50 +math.sqrt(2 * math.pow(50, 2)), global_var.k4[1][0] + const.CAM_IMG_OFS + 50, global_var.k4[1][1] + 50)
			

					if(global_var.max_power - power4 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)
						painter3.drawPath(path4_4)
						painter3.drawPath(path4_5)
			
					if(global_var.max_power - power4 > 0 and global_var.max_power - power4 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)
						painter3.drawPath(path4_4)
	   
					if(global_var.max_power - power4 > 0.5 and global_var.max_power - power4 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)

					if(global_var.max_power - power4 > 1.0 and global_var.max_power - power4 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)

					if(global_var.max_power - power4 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path4_1)
			
			#直接選択
			for i in range(len(global_var.vx)):
				if(global_var.dflag == True):
					painter2.setPen(pen4)
					painter2.setFont(QtGui.QFont('Decorative', 14))
					painter2.drawText(global_var.vx[i] - 20, 30, self.text1)
	
			#描画処理
			for j in range(len(global_var.a)):
				for k in range(1, len(global_var.a[j][0])):
					if (global_var.cflag == True):
						painter2.setPen(pen5)
						painter2.drawLine(global_var.a[j][0][k-1], global_var.a[j][1][k-1], global_var.a[j][0][k], global_var.a[j][1][k])

			#定位情報表示
			if(global_var.lflag == True):
				for h in range(len(global_var.locSrcList)):
					#画面外情報
					if(global_var.locSrcList[h].theta < -30 and global_var.locSrcList[h].theta >= -180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220 + const.CAM_IMG_OFS, 440, self.text2)

					elif(global_var.locSrcList[h].theta >= 30 and global_var.locSrcList[h].theta < 180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220 + const.CAM_IMG_OFS, 440, self.text2)


	#音源情報を元に分離音声視聴ボタンを作成
	def showListenButton(self,locSrc):

		#画面内のボタンを表示
		def showOnImageButton(btnList):
			debugTime = time.time()
			#位置が被るボタンが無いか探索
			showButtonFlag = True
			for bindex in range(len(btnList)):
				if btnList[bindex].isVisible():
					if abs(locSrc.x_onImage - btnList[bindex].geometry().left()) < const.ONIMG_AUTO_BTN_MGN:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						if btnList[bindex].theta < 0:
							btnList[bindex].setGeometry(locSrc.x_onImage + const.CAM_IMG_OFS - const.ONIMG_LIS_BTN_WID/2 -60,const.ONIMG_LIS_BTN_Y,const.ONIMG_LIS_BTN_WID,const.ONIMG_LIS_BTN_HT)
						else:
							btnList[bindex].setGeometry(locSrc.x_onImage + const.CAM_IMG_OFS - const.ONIMG_LIS_BTN_WID/2,const.ONIMG_LIS_BTN_Y,const.ONIMG_LIS_BTN_WID,const.ONIMG_LIS_BTN_HT)
						btnList[bindex].setVisible(True)
						btnList[bindex].theta = locSrc.theta
						break



		#画面外のボタンを表示
		def showButton(btnList,xaxis):
			debugTime = time.time()
			#位置が被るボタンが無いか探索
			showButtonFlag = True
			for bindex in range(len(btnList)):
				if btnList[bindex].isVisible():
					if abs((locSrc.y_arrow + const.ARW_HT/2) -(btnList[bindex].geometry().top() + const.AUTO_ROT_BTN_HT/2)) < const.AUTO_BTN_MGN:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						btnList[bindex].setGeometry(xaxis,locSrc.y_arrow + (const.ARW_HT - const.AUTO_ROT_BTN_HT)/2,const.AUTO_ROT_BTN_WID,const.AUTO_ROT_BTN_HT)
						btnList[bindex].setVisible(True)
						btnList[bindex].theta = locSrc.theta
						break


		#画面内
		if locSrc.onImageFlag is True:
			if locSrc.theta > 0:
				showOnImageButton(global_var.onImageLeftListenBtnList)
			else:
				showOnImageButton(global_var.onImageRightListenBtnList)
		#画面外
		else:
			if locSrc.theta > 0:
				showButton(global_var.leftListenBtnList,const.L_LIS_BTN_X)
			else:
				showButton(global_var.rightListenBtnList,const.R_LIS_BTN_X)


	#音源情報を元に自動回転ボタンを作成
	def showAutoRotateButton(self,locSrc):

		#画面外のボタンを表示
		def showButton(btnList,xaxis):
			#位置が被るボタンが無いか探索
			showButtonFlag = True
			for bindex in range(len(btnList)):
				if btnList[bindex].isVisible():
					if abs((locSrc.y_arrow + const.ARW_HT/2) -(btnList[bindex].geometry().top() + const.AUTO_ROT_BTN_HT/2)) < const.AUTO_BTN_MGN:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						btnList[bindex].setGeometry(xaxis,locSrc.y_arrow + (const.ARW_HT - const.AUTO_ROT_BTN_HT)/2,const.AUTO_ROT_BTN_WID,const.AUTO_ROT_BTN_HT)
						
						btnList[bindex].setVisible(True)
						btnList[bindex].timeout = abs(const.TO_PER_THETA * locSrc.theta)
						break

		if locSrc.theta > 0:
			showButton(global_var.leftAutoRotBtnList,const.AUTO_L_ROT_BTN_X)
		else:
			showButton(global_var.rightAutoRotBtnList,const.AUTO_R_ROT_BTN_X)

	#音源情報を元に分離音声視聴ボタンを削除
	def hideListenButton(self,srcList,vanSrcList):
		debugTime = time.time()
	
		leftSrcList = []
		rightSrcList = []
		onImageLeftSrcList = []
		onImageRightSrcList = []
		
		debugTime = 0

		def hideButton(sortSrcList,btnList,onImageFlag):
			for bindex in range(len(btnList)):
				setInvisibleFlag = True
				if btnList[bindex].isVisible():

					debugTime = time.time()
					for sindex in range(len(sortSrcList)):
						if onImageFlag is True:
							if abs((sortSrcList[sindex].x_onImage + const.CAM_IMG_OFS) -btnList[bindex].geometry().left()) < const.ONIMG_AUTO_BTN_MGN:
								setInvisibleFlag = False
								break
						else:
							if abs(sortSrcList[sindex].y_arrow -btnList[bindex].geometry().top()) < const.AUTO_BTN_MGN:
								setInvisibleFlag = False
								break
					if setInvisibleFlag:
						btnList[bindex].setVisible(False)

		def sortSourceLR(srcList):
			for i in range(len(srcList)):
				if srcList[i].onImageFlag is True:
					if srcList[i].theta > 0:
						onImageLeftSrcList.append(srcList[i])
					else:
						onImageRightSrcList.append(srcList[i])
				else:
					if srcList[i].theta > 0:
						leftSrcList.append(srcList[i])
					else:
						rightSrcList.append(srcList[i])



		#左右のソースを仕分け
		sortSourceLR(srcList)
		sortSourceLR(vanSrcList)


		#不要なボタンをかくす
		hideButton(leftSrcList,global_var.leftListenBtnList,False)
		hideButton(rightSrcList,global_var.rightListenBtnList,False)
		hideButton(onImageLeftSrcList,global_var.onImageLeftListenBtnList,True)
		hideButton(onImageRightSrcList,global_var.onImageRightListenBtnList,True)


	#音源情報を元に自動回転ボタンを削除
	def hideAutoRotateButton(self,srcList,vanSrcList):
		leftSrcList = []
		rightSrcList = []

		def hideButton(srcList,btnList):
			for bindex in range(len(btnList)):
				setInvisibleFlag = True
				if btnList[bindex].isVisible():
					for sindex in range(len(srcList)):
						if not srcList[sindex].onImageFlag:
							if abs(srcList[sindex].y_arrow -btnList[bindex].geometry().top()) < const.AUTO_BTN_MGN:
								setInvisibleFlag = False
								break
					if setInvisibleFlag:
						btnList[bindex].setVisible(False)

		def sortSourceLR(srcList):
			for i in range(len(srcList)):
				if srcList[i].theta > 0:
					leftSrcList.append(srcList[i])
				else:
					rightSrcList.append(srcList[i])

		#左右のソースを仕分け
		sortSourceLR(srcList)
		sortSourceLR(vanSrcList)

		#不要なボタンをかくす
		hideButton(leftSrcList,global_var.leftAutoRotBtnList)
		hideButton(rightSrcList,global_var.rightAutoRotBtnList)

	def paintListenRange(self):
		if global_var.listenSeparateSoundFlag:
			qp = QtGui.QPainter()
			qp.begin(self)
			color = QtGui.QColor(255, 255, 0, 50)
			qp.setBrush(color)
			qp.drawRect(self.getPaintRect(self.listenRangeStartX,self.listenRangeEndX))

	#描画処理全般（カメラ画像、情報提示）
	def paintEvent(self, e):
		tmpLocSrcList = global_var.locSrcList[:]
		tmpVanLocSrcList= global_var.vanLocSrcList[:]

		painter1 = QtGui.QPainter(self)

		#矢印用ペインタ
		arrow_right_x1_painter = QtGui.QPainter(self)
		arrow_right_x2_painter = QtGui.QPainter(self)
		arrow_right_x3_painter = QtGui.QPainter(self)
		arrow_left_x1_painter = QtGui.QPainter(self)
		arrow_left_x2_painter = QtGui.QPainter(self)
		arrow_left_x3_painter = QtGui.QPainter(self)
		
		#キャラクター用ペインタ
		character_right_painter = QtGui.QPainter(self)
		character_left_painter = QtGui.QPainter(self)


		#カメライメージ描画
		if self._frame is not None:
			painter1.drawImage(QtCore.QPoint(const.CAM_IMG_OFS, 0), OpenCVQImage(self._frame))

		self.paintListenRange()

		#口付近に表示する音の波を描画
		self.paintVoiceWave(e)

		#manualボタンがdisableになっていたら、ableに戻すか判定
		if not global_var.manualButtonAbleFlag :
			if time.time() > global_var.manualFinDisableTime:
				global_var.manualButtonAbleFlag  = True
				button.switchManBtnAble(True)


		#不要ボタンを非表示化
		#自動回転ボタン
		self.hideAutoRotateButton(tmpLocSrcList,tmpVanLocSrcList)
		#左分離音声視聴ボタン
		self.hideListenButton(tmpLocSrcList,tmpVanLocSrcList)


		#ボタン描画
		for index in range(len(tmpLocSrcList)):
			if tmpLocSrcList[index].onImageFlag:#画面内
				#分離音声視聴ボタン
				self.showListenButton(tmpLocSrcList[index])

			else:#画面外
				if tmpLocSrcList[index].theta > 0:#左側
					#矢印を描画
					if tmpLocSrcList[index].powerCode == const.STRONG_POW_CODE:
						self.arrow_left_x1_point = QtCore.QPoint(const.ARW_L_1_X,tmpLocSrcList[index].y_arrow)
						arrow_left_x1_painter.drawImage(self.arrow_left_x1_point,self.arrow_left_x1_image)
						button_xaxis = const.AUTO_L_ROT_BTN_X
					elif tmpLocSrcList[index].powerCode == const.MEDIUM_POW_CODE:
						self.arrow_left_x2_point = QtCore.QPoint(const.ARW_L_2_X,tmpLocSrcList[index].y_arrow)
						arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_image)
						button_xaxis = const.AUTO_L_ROT_BTN_X
					else:
						self.arrow_left_x3_point = QtCore.QPoint(const.ARW_L_3_X,tmpLocSrcList[index].y_arrow)
						arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_image)
						button_xaxis = const.AUTO_L_ROT_BTN_X

					#回転ボタン
					self.showAutoRotateButton(tmpLocSrcList[index])

					#分離音声視聴ボタン
					self.showListenButton(tmpLocSrcList[index])


				else:#右側
					button_xaxis = 0
					if tmpLocSrcList[index].powerCode == const.STRONG_POW_CODE:
						self.arrow_right_x1_point = QtCore.QPoint(const.ARW_R_X,tmpLocSrcList[index].y_arrow)
						arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_image)
					elif tmpLocSrcList[index].powerCode == const.MEDIUM_POW_CODE:
						self.arrow_right_x2_point = QtCore.QPoint(const.ARW_R_X,tmpLocSrcList[index].y_arrow)
						arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_image)
					else:
						self.arrow_right_x3_point = QtCore.QPoint(const.ARW_R_X,tmpLocSrcList[index].y_arrow)
						arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_image)

					#回転ボタン
					self.showAutoRotateButton(tmpLocSrcList[index])

					#分離音声視聴ボタン
					self.showListenButton(tmpLocSrcList[index])


		#薄矢印描画
		for index in range(len(tmpVanLocSrcList)):
			if not tmpVanLocSrcList[index].onImageFlag:#画面内
				if tmpVanLocSrcList[index].theta > 0:
					if tmpVanLocSrcList[index].powerCode == const.STRONG_POW_CODE:
						self.arrow_left_x1_point = QtCore.QPoint(const.ARW_L_1_X,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x1_painter.drawImage(self.arrow_left_x1_point,self.arrow_left_x1_pale_image)
					elif tmpVanLocSrcList[index].powerCode == const.MEDIUM_POW_CODE:
						self.arrow_left_x2_point = QtCore.QPoint(const.ARW_L_2_X,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_pale_image)
					else:
						self.arrow_left_x3_point = QtCore.QPoint(const.ARW_L_3_X,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_pale_image)
					
				else:
					if tmpVanLocSrcList[index].powerCode == const.STRONG_POW_CODE:
						self.arrow_right_x1_point = QtCore.QPoint(const.ARW_R_X,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_pale_image)
					elif tmpVanLocSrcList[index].powerCode == const.MEDIUM_POW_CODE:
						self.arrow_right_x2_point = QtCore.QPoint(const.ARW_R_X,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_pale_image)
					else:
						self.arrow_right_x3_point = QtCore.QPoint(const.ARW_R_X,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_pale_image)
	
	
	#クリックが正面映像上でなければX座標を補正する
	def getOnImageX(self,x):
		if x < const.CAM_IMG_OFS:
			return const.CAM_IMG_OFS
		elif x > (const.CAM_IMG_OFS + const.CAM_IMG_WID):
			return const.CAM_IMG_OFS + const.CAM_IMG_WID
		else:
			return x


	#def ifOnSideImage(x,y):

	def mousePressEvent(self,event):
		self.listenRangeStartX = self.getOnImageX(event.x())
		self.listenRangeEndX =  self.listenRangeStartX
		global_var.listenSeparateSoundFlag = True
		#p = QtGui.QPixmap.grabWindow(self.winId())
		#p.save("scrshot"+str(time.time()),"png")

	def mouseDoubleClickEvent(self,event):
		format_loc_src_microcone.listenWholeSound()
		global_var.listenSeparateSoundFlag = False

	def mouseMoveEvent(self,event):
		self.listenRangeEndX = self.getOnImageX(event.x())

	def mouseReleaseEvent(self, e):
		#座標を角度に変換してグローバル変数にセット
		print "startx:"+str(self.listenRangeStartX - const.CAM_IMG_OFS)
		print "endx:" + str(self.listenRangeEndX - const.CAM_IMG_OFS)
		if math.fabs(self.listenRangeEndX - self.listenRangeStartX) > const.IGNOR_PIX_THR:
			format_loc_src_microcone.setListenAngles(self.listenRangeStartX - const.CAM_IMG_OFS,self.listenRangeEndX - const.CAM_IMG_OFS)
			format_loc_src_microcone.listenSeparateSound()
			global_var.listenSeparateSoundFlag = True
		else:
			self.listenRangeEndX = self.listenRangeStartX
			global_var.listenSeparateSoundFlag = False
			format_loc_src_microcone.listenWholeSound()
		print "separate"
		print "from:" + str(global_var.listenRangeStartAngle)
		print "to:" + str(global_var.listenRangeEndAngle)

	def keyPressEvent(self,event):
		key = event.key()
		command = None
		if key == Qt.Key_6:
			command = const.R_ROT_CMD
		elif key == Qt.Key_4:
			command = const.L_ROT_CMD
		elif key == Qt.Key_8:
			command = const.FWD_CMD
		elif key == Qt.Key_2:
			command = const.BACK_CMD
		sendCommand(command,const.KEY_MAN_ROT_TO)
	
	def getPaintRect(self,startX,endX):
		absRange = math.fabs(endX - startX)
		
		if startX < endX:
			return QRect(startX,0,absRange,const.CAM_IMG_HT)
		else:
			return QRect(endX,0,absRange,const.CAM_IMG_HT)


#ウィンドウ全体
class MainWindow(QtGui.QMainWindow):
	
	def __init__(self):
		super(MainWindow, self).__init__()
		
		#backGroundColor
		#self.setStyleSheet("background-color: white")#背景色を白に

		#MenuBar
		self.exit_menu = self.menuBar().addMenu("&Exit")
		self.exit = self.exit_menu.addAction(u"終了...", QtGui.QApplication.quit)
		self.exit.setShortcut("Ctrl+C")
		self.exit.setStatusTip(u"アプリケーションを終了します")

		#CameraWidget
		cameraDevice = CameraDevice(mirrored = True)
		self.camerawidget = CameraWidget(cameraDevice)
		self.setCentralWidget(self.camerawidget)
		self.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)
		#self.setAutoFillBackground(False)
		self.statusBar().showMessage("Welcome to UI-ALT!")
		#self.click.triggered.connect(self.Click)

#全体の処理をスレッド化
class QtThread():

	def main():
		app = QtGui.QApplication(sys.argv)
		window = MainWindow()
		window.setWindowTitle('UI-ALT')
		window.show()
		app.exec_()

	t = threading.Thread(None, main, 'Qt')
	t.setDaemon(1)
	t.start()

	#Subscriber 



#トピック購読処理
def subscriber():
	rospy.init_node('UIALT', anonymous = True)
	kinect_tf.subscriber()
	format_loc_src_microcone.subscriber()
	rospy.spin()

#メイン関数
if __name__ == '__main__':
	t = QtThread()
	subscriber()
