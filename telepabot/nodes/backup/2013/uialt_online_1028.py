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
import config


#カメラ映像の大きさと画面内の表示位置

CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480
CAMERA_IMAGE_OFFSET = 400

#定数(言語仕様では定数は無く、書き換え可能なので注意)
#ウィンドウの大きさ
WINDOW_WIDTH = CAMERA_IMAGE_WIDTH + CAMERA_IMAGE_OFFSET*2
WINDOW_HEIGHT = 780

#ボタンの大きさ
LISTEN_BUTTON_WIDTH = 50
LISTEN_BUTTON_HEIGHT = 50
AUTO_ROTATE_BUTTON_WIDTH = 50
AUTO_ROTATE_BUTTON_HEIGHT = 50

#ボタン間の距離
BUTTON_MARGIN = 20

#矢印の大きさ
ARROW_HEIGHT = 80
ARROW_WIDTH_X1 = 100
ARROW_WIDTH_X2 = 150
ARROW_WIDTH_X3 = 200

#右向き矢印の座標
ARROW_RIGHT_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + BUTTON_MARGIN*3 + AUTO_ROTATE_BUTTON_WIDTH + LISTEN_BUTTON_WIDTH


#左向き矢印の座標
ARROW_LEFT_X1_XAXIS = 20 + ARROW_WIDTH_X3 - ARROW_WIDTH_X1
ARROW_LEFT_X2_XAXIS = 20 + ARROW_WIDTH_X3 - ARROW_WIDTH_X2
ARROW_LEFT_X3_XAXIS = 20

#矢印表示の角度閾値
ARROW_X1_THRESHOLD = 28.5
ARROW_X2_THRESHOLD = 50
ARROW_X3_THRESHOLD = 75


#画像内の角度領域
IMAGE_BOUNDARY_THETA = 28.5



#前進ボタンの位置・大きさ
MANUAL_FORWARD_WIDTH = 50
MANUAL_FORWARD_HEIGHT = 50
MANUAL_FORWARD_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH/2 - MANUAL_FORWARD_WIDTH/2
#590
MANUAL_FORWARD_YAXIS = CAMERA_IMAGE_HEIGHT + 50

#後退ボタンの位置・大きさ
MANUAL_BACK_WIDTH = 50
MANUAL_BACK_HEIGHT = 50
MANUAL_BACK_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH/2 - MANUAL_BACK_WIDTH/2
MANUAL_BACK_YAXIS = CAMERA_IMAGE_HEIGHT + 190

#左回転ボタンの位置・大きさ
MANUAL_LEFT_ROTATE_BUTTON_XAXIS = MANUAL_FORWARD_XAXIS - 130
MANUAL_LEFT_ROTATE_BUTTON_YAXIS = CAMERA_IMAGE_HEIGHT + 120
MANUAL_LEFT_ROTATE_BUTTON_WIDTH = 50
MANUAL_LEFT_ROTATE_BUTTON_HEIGHT = 50

#右回転ボタンの位置・大きさ
MANUAL_RIGHT_ROTATE_BUTTON_XAXIS = MANUAL_FORWARD_XAXIS + 130
MANUAL_RIGHT_ROTATE_BUTTON_YAXIS = MANUAL_LEFT_ROTATE_BUTTON_YAXIS
MANUAL_RIGHT_ROTATE_BUTTON_WIDTH = 50
MANUAL_RIGHT_ROTATE_BUTTON_HEIGHT = 50

#音声分離リセットボタン
ALL_LISTEN_BUTTON_WIDTH = 50
ALL_LISTEN_BUTTON_HEIGHT = 50
ALL_LISTEN_BUTTON_XAXIS = MANUAL_FORWARD_XAXIS
ALL_LISTEN_BUTTON_YAXIS = MANUAL_LEFT_ROTATE_BUTTON_YAXIS

#キャラクタアイコンの大きさ
#CHARACTER_WIDTH = 80
#CHARACTER_HEIGHT = 80
ONIMAGE_LISTEN_BUTTON_WIDTH = 50
ONIMAGE_LISTEN_BUTTON_HEIGHT = 50

#ボタン識別コード
RIGHT_ROTATE_BUTTON_CODE = 0
LEFT_ROTATE_BUTTON_CODE = 1
FORWARD_BUTTON_CODE = 2
BACK_BUTTON_CODE = 3

##################画像データ################################
#ボタンアイコン
AUTO_LEFT_ROTATE_BUTTON_ICON = None
AUTO_RIGHT_ROTATE_BUTTON_ICON = None

#右矢印x1-x3
ARROW_RIGHT_X1_IMAGE = None
ARROW_RIGHT_X2_IMAGE = None
ARROW_RIGHT_X3_IMAGE = None

#左矢印x1-x3
ARROW_LEFT_X1_IMAGE = None
ARROW_LEFT_X2_IMAGE = None
ARROW_LEFT_X3_IMAGE = None

#右向きキャラ
CHARACTER_RIGHT_IMAGE = None

#左向きキャラ
CHARACTER_LEFT_IMAGE = None



#矢印表示位置を決める際の角度の刻み幅,それごとに動くピクセル
THETA_STRIDE = 10
MAX_THETA = 180#取れる音源の角度の限界
PIXEL_BY_STRIDE = int(CAMERA_IMAGE_WIDTH * THETA_STRIDE / MAX_THETA)

#消えた音源ソース情報を保持する秒数
VANISHED_SOURCE_KEEP_SEC = 3.0

#自動回転ボタンの数
AUTO_ROTATE_BUTTON_NUM = 6


#分離音声視聴ボタン
LEFT_LISTEN_BUTTON_XAXIS = CAMERA_IMAGE_OFFSET - AUTO_ROTATE_BUTTON_WIDTH - BUTTON_MARGIN
RIGHT_LISTEN_BUTTON_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + BUTTON_MARGIN
LISTEN_BUTTON_NUM = 6
ONIMAGE_LISTEN_BUTTON_YAXIS = 50


#自動回転ボタン
AUTO_LEFT_ROTATE_BUTTON_XAXIS = CAMERA_IMAGE_OFFSET - AUTO_ROTATE_BUTTON_WIDTH - LISTEN_BUTTON_WIDTH - BUTTON_MARGIN*2
AUTO_RIGHT_ROTATE_BUTTON_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + LISTEN_BUTTON_WIDTH + BUTTON_MARGIN*2
#AUTO_RIGHT_ROTATE_BUTTON_X1_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + ARROW_WIDTH_X1 + 20
#AUTO_RIGHT_ROTATE_BUTTON_X2_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + ARROW_WIDTH_X2 + 20
#AUTO_RIGHT_ROTATE_BUTTON_X3_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + ARROW_WIDTH_X3 + 20
SPACE_BETWEEN_AUTO_BUTTON = 60
ONIMAGE_SPACE_BETWEEN_AUTO_BUTTON = 100
OFFSET_FROM_ARROW_TO_BUTTON = 20

#staticに使いたいインスタンス(今のところコマンド送信用)
LEFT_ROTATE_COMMAND = Twist()
LEFT_ROTATE_COMMAND.angular.z = 0.88
RIGHT_ROTATE_COMMAND = Twist()
RIGHT_ROTATE_COMMAND.angular.z = -0.88
FORWARD_COMMAND = Twist()
FORWARD_COMMAND.linear.x = 0.1
BACK_COMMAND = Twist()
BACK_COMMAND.linear.x = -0.1
#COMMAND_PUBLISHER = rospy.Publisher('/cmd_vel', Twist)

#turtlebotにタイムアウトを設定する用
SET_TIMEOUT_STR = "rosparam set /turtlebot_node/cmd_vel_timeout "
MANUAL_ROTATE_TIMEOUT = 0.4
KEY_MANUAL_ROTATE_TIMEOUT = 0.5
JOY_MANUAL_ROTATE_TIMEOUT = 0.4
DEFAULT_TIMEOUT = 0.0
TIMEOUT_PER_THETA = 0.022222
SEND_CMD_OVERHEAD = 1.0

#音源パワー分類用
WEAK_POWER_CODE = 0
MEDIUM_POWER_CODE = 1
STRONG_POWER_CODE = 2
#STRONG_POWER_THRESHOLD = 33.6
#MEDIUM_POWER_THRESHOLD = 33.3
STRONG_POWER_THRESHOLD = 0.5
MEDIUM_POWER_THRESHOLD = 1

"""
#グローバル変数
debugFlag = True
flag = []
dflag = True
cflag = True
lflag = True
sflag = True
vx = []
vy = []
k1 = []
k2 = []
k3 = []
k4 = []
max_theta = []
min_theta = []
max_power = 0
d_theta = []
id = []
xarray = []
yarray = []
a = []
locSrcList = []#harkから得た定位音源情報を整形して入れる配列
prevLocSrcList = []#直前の定位音源情報
vanLocSrcList= []#消えた音源の情報をしばらく入れる配列
msg_select = []
cv_image = None
tfframe = []
joy = None
harkMsg = HarkSource()#HarkSource型のままのメッセージ
prev_msg = HarkSource()
leftAutoRotBtnList = []#左自動回転のボタンリスト
rightAutoRotBtnList = []#右自動回転のボタンリスト
leftListenBtnList = []#左分離音声視聴ボタンリスト
rightListenBtnList = []#右分離音声視聴ボタンリスト
onImageLeftListenBtnList = []#画面上・左分離音声視聴ボタンリスト
onImageRightListenBtnList = []#画面上・右分離音声視聴ボタンリスト

#マニュアルボタン
rightRotateButton = None
leftRotateButton = None
forwardButton = None
backButton = None
allListenButton = None

#マニュアルボタンのable/disable管理
manualButtonAbleFlag = True
manualFinDisableTime = 0
"""

FRAMES = [
	'head',
	'neck',
	'torso',
	'left_shoulder',
	'right_shoulder'
	]


#定位した音源情報を格納するクラス
class LocSrc():
	def __init__(self,srcId,theta,power,x_onImage,y_arrow,onImageFlag,powerCode):
		self.srcId, self.theta, self.power, self.x_onImage, self.y_arrow, self.onImageFlag,self.deletedTime,self.powerCode = srcId,theta,power,x_onImage,y_arrow,onImageFlag,0,powerCode

#角度しきい値を入れるクラス
#class threshold():
#	def __init__(self,min_theta,max_theta):
#		self.min_theta,self.max_theta = min_theta,max_theta

#音源の角度を引数にとり、付与するIDを返す
def getSourceDirectionId(theta):
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




#画面内の音を選択する用
def src_select():

	global d_theta
	global locSrcList
	global min_theta
	global max_theta
	global id
	global sflag
	global dflag
	global cflag
	global msg_select

	msg_select = HarkSource()
	msg_select.exist_src_num = 0
	trigger = rospy.Publisher('Trigger', Bool)
	pub_select = rospy.Publisher('SourceSelect', HarkSource)
	select = False
	flag = False

	#何も選択されていない場合，生データ再生
	if(sflag == False):

		msg_select.src = []
		trigger.publish(select)
		msg_select.exist_src_num = 0

	#選択された場合
	if(sflag == True and flag == False):

		if(cflag == True):

			for i in range(len(max_theta)):
				append = HarkSourceVal()
				if((min_theta[i] + max_theta[i]) / 2 >= -10 and (min_theta[i] + max_theta[i]) / 2 < 10):
					append.id = 0
				if((min_theta[i] + max_theta[i]) / 2 >= -30 and (min_theta[i] + max_theta[i]) / 2 < -10):
					append.id = 2
				if((min_theta[i] + max_theta[i]) / 2 >= 10 and (min_theta[i] + max_theta[i]) / 2 < 30):
					append.id = 1

			msg_select.src.append(append)
			flag = True
			msg_select.exist_src_num = len(max_theta)
			select = True

	if(dflag == True):
		for i in range(len(d_theta)):
			append = HarkSourceVal()
			if(d_theta[i] >= -10 and d_theta[i] < 10):
				append.id = 0
			if(d_theta[i] >= -30 and d_theta[i] < -10):
				append.id = 1
			if(d_theta[i] >= 10 and d_theta[i] < 30):
				append.id = 2
			msg_select.src.append(append)
			flag = True
			msg_select.exist_src_num = len(d_theta)
			select = True
	
	pub_select.publish(msg_select)
	r = rospy.Rate(3)
	r.sleep()
	trigger.publish(select)

#joystickの操作に従いタートルボットを動かす
#def joy_callback(data):
#	if data is not None:
#		print "dataNotNone"
#		command = Twist()
#		command.angular.z = data.axes[1]
#		print "angularz"
#		print command.angular.z
#		sendCommand(command,JOY_MANUAL_ROTATE_TIMEOUT)


#Show localization information
def localization_callback(data):

	#print "local_callback"

	global locSrcList
	global prevLocSrcList
	global max_power
	global prev_msg
	global vanLocSrcList
	global harkMsg
	
	harkMsg = copy.deepcopy(data)

	locSrcList = []

	pub = rospy.Publisher('DetectedSource', HarkSource)
	msg = HarkSource()

	if len(data.src) > 0:
		max_power = data.src[0].power
		for j in range(len(data.src)):#音源リスト内の最大パワーを求める
			if data.src[j].power - max_power > 0:
				max_power = data.src[j].power


	for i in range(len(data.src)):#harkSourceに画像上のx座標情報を追加してlocSrcListに格納
		if(data.src[i].theta >= -IMAGE_BOUNDARY_THETA and data.src[i].theta <= IMAGE_BOUNDARY_THETA):#映像視野内
			x_onImage = (CAMERA_IMAGE_WIDTH / 2)*(1-(math.tan(math.radians(data.src[i].theta))/math.tan(math.radians(IMAGE_BOUNDARY_THETA))))
			onImageFlag = True
			y_arrow = -100
		else:#画面外
			if(data.src[i].theta > 0):
				y_arrow = PIXEL_BY_STRIDE*(int( (data.src[i].theta - IMAGE_BOUNDARY_THETA) / THETA_STRIDE ))
			else:
				y_arrow = -PIXEL_BY_STRIDE*(int( (data.src[i].theta + IMAGE_BOUNDARY_THETA) / THETA_STRIDE ))
			x_onImage = -100
			onImageFlag = False
			#flagoos = True

#		if data.src[i].power > STRONG_POWER_THRESHOLD:
#			powerCode = STRONG_POWER_CODE
#		elif data.src[i].power > MEDIUM_POWER_THRESHOLD:
#			powerCode = MEDIUM_POWER_CODE
#		else:
#			powerCode = WEAK_POWER_CODE

		if (max_power - data.src[i].power) < STRONG_POWER_THRESHOLD:
			powerCode = STRONG_POWER_CODE
		elif (max_power - data.src[i].power) < MEDIUM_POWER_THRESHOLD:
			powerCode = MEDIUM_POWER_CODE
		else:
			powerCode = WEAK_POWER_CODE

		src = LocSrc(data.src[i].id, data.src[i].theta, data.src[i].power,x_onImage,y_arrow,onImageFlag,powerCode)
		locSrcList.append(src)



	#消えた音源があればリストに格納
	for prevIndex in range(len(prevLocSrcList)):
		#if not prevLocSrcList[prevIndex].onImageFlag:
		flag = False
		for locIndex in range(len(locSrcList)):
			if(prevLocSrcList[prevIndex].srcId == locSrcList[locIndex].srcId):
				flag = True
		if not flag:
			prevLocSrcList[prevIndex].deletedTime = time.time()
			vanLocSrcList.append(prevLocSrcList[prevIndex])

	prevLocSrcList = locSrcList[:]#リストの中身をコピー

	#消えて五秒経過した音源は削除
	tmpList = []
	for index in range(len(vanLocSrcList)):
		if (time.time() - vanLocSrcList[index].deletedTime) < VANISHED_SOURCE_KEEP_SEC:
			tmpList.append(vanLocSrcList[index])
	vanLocSrcList= tmpList[:]

	#存在する音源と角度が被っていたら消えた音源の情報は削除
	tmpList = []
	for vanishIndex in range(len(vanLocSrcList)):
		deleteFlag = False
		for locIndex in range(len(locSrcList)):
			if locSrcList[locIndex].y_arrow == vanLocSrcList[vanishIndex].y_arrow:
				deleteFlag = True
		if not deleteFlag:
			tmpList.append(vanLocSrcList[vanishIndex])
	vanLocSrcList= tmpList[:]


	#分離用の処理？
	for k in range(len(data.src)):
		msg_val = HarkSourceVal()
		if(data.src[k].theta >= -10 and data.src[k].theta < 10):
			msg_val.id = 0
		elif(data.src[k].theta >= -30 and data.src[k].theta < -10):
			msg_val.id = 1
		elif(data.src[k].theta >= 10 and data.src[k].theta <= 30):
			msg_val.id = 2
		elif(data.src[k].theta > -75 and data.src[k].theta < -30):
			msg_val.id = 3
		elif(data.src[k].theta > -105 and data.src[k].theta <= -75):
			msg_val.id = 4
		elif(data.src[k].theta > -135 and data.src[k].theta <= -105):
			msg_val.id = 5
		elif(data.src[k].theta > -165 and data.src[k].theta <= -135):
			msg_val.id = 6
		elif(data.src[k].theta > 30 and data.src[k].theta <= 75):
			msg_val.id = 7
		elif(data.src[k].theta > 75 and data.src[k].theta <= 105):
			msg_val.id = 8
		elif(data.src[k].theta > 105 and data.src[k].theta <= 135):
			msg_val.id = 9
		elif(data.src[k].theta > 135 and data.src[k].theta <= 165):
			msg_val.id = 10
		elif((data.src[k].theta > 165 and data.src[k].theta <= 180) or (data.src[k].theta >= -180 and data.src[k].theta <= -165)):
			msg_val.id = 11
		
		msg_val.x  = data.src[k].x
		msg_val.y  = data.src[k].y
		msg_val.theta = data.src[k].theta
		msg.src.append(msg_val)
		msg.exist_src_num += 1
	
	for l in range(len(prev_msg.src)):
		flag = False
		for n in range(len(msg.src)):
			if(prev_msg.src[l].id == msg.src[n].id):
				flag = True
		if(flag == False):
			msg.src.append(prev_msg.src[l])
			msg.exist_src_num += 1
	pub.publish(msg)
	prev_msg = msg


#kinectのトラッキングデータをグローバル配列に格納
def tf_callback1(data):
	
	global k1
	k1 = []

	for i in range(len(data.src)):
		k1.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	print "k1"
	print k1
	
def tf_callback2(data):

	global k2
	k2 = []

	for i in range(len(data.src)):
		k2.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	
def tf_callback3(data):

	global k3
	k3 = []

	for i in range(len(data.src)):
		k3.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	
def tf_callback4(data):

	global k4
	k4 = []

	for i in range(len(data.src)):
		k4.append((data.src[i].x, data.src[i].y, data.src[i].theta))

#マニュアル操作ボタンのable/disable切り替え
def switchManualButtonAble(flag):
	leftRotateButton.setEnabled(flag)
	rightRotateButton.setEnabled(flag)
	forwardButton.setEnabled(flag)
	backButton.setEnabled(flag)


#turtlebotへコマンドを送信
def sendCommand(command,timeout):
	os.system(SET_TIMEOUT_STR + str(timeout))
	pub = rospy.Publisher('/cmd_vel', Twist)
	pub.publish(command)
	rospy.loginfo(command)


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
		#self.setAutoRepeatInterval(1000)
		self.clicked.connect(self.sendManualCommand)
		self._state = 0
		#self.setStyleSheet('QPushButton {color: blue}')
#		if code == RIGHT_ROTATE_BUTTON_CODE:
#			self.keyPressEvent()
	
#	def keyPressed(self,event):
#		setDown(self,)
#		key = event.key()
#		if key == Qt.Key_Right:
#			
#		elif key == Qt.Key_Left:

#		elif key == Qt.Key_Up:

#		elif key == Qt.Key_Down:

	def setButtonCode(self,code):#ボタンの種類(引数)毎に送るコマンドを設定
		self.pub = rospy.Publisher('/cmd_vel', Twist)
		self.command = Twist()
		if code == RIGHT_ROTATE_BUTTON_CODE:
			self.command = RIGHT_ROTATE_COMMAND
		elif code == LEFT_ROTATE_BUTTON_CODE:
			self.command = LEFT_ROTATE_COMMAND
		elif code == FORWARD_BUTTON_CODE:
			self.command = FORWARD_COMMAND
		elif code == BACK_BUTTON_CODE:
			self.command = BACK_COMMAND

	def sendManualCommand(self):#turtlebotに回転・移動コマンドを送信
		if self.isDown():#押している間コマンドを送り続ける
			sendCommand(self.command,MANUAL_ROTATE_TIMEOUT)
			print 'repeat'
		elif self._state == 1:
			self._state = 0
			os.system(SET_TIMEOUT_STR + str(DEFAULT_TIMEOUT))
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
		self.setGeometry(0,0,AUTO_ROTATE_BUTTON_WIDTH,AUTO_ROTATE_BUTTON_HEIGHT)
		self.setIconSize(QtCore.QSize(AUTO_ROTATE_BUTTON_WIDTH,AUTO_ROTATE_BUTTON_HEIGHT))
		self.setVisible(False)

	def sendAutoCommand(self):#turtlebotに回転・移動コマンドを送信
		global manualButtonAbleFlag
		global manualFinDisableTime
		manualButtonAbleFlag = False
		manualFinDisableTime = time.time() + self.timeout + SEND_CMD_OVERHEAD
		switchManualButtonAble(False)
		sendCommand(self.command,self.timeout)

#分離音声視聴ボタン
class ListenButton(QtGui.QPushButton):
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.clicked.connect(self.listen)
		self.theta = 0
		self.setGeometry(0,0,LISTEN_BUTTON_WIDTH,LISTEN_BUTTON_HEIGHT)
		self.setIconSize(QtCore.QSize(LISTEN_BUTTON_WIDTH,LISTEN_BUTTON_HEIGHT))
		self.setVisible(False)
	
	def listen(self):
		msg_select = HarkSource()
		msg_select.exist_src_num = 1
		trigger = rospy.Publisher('Trigger', Bool)
		pub_select = rospy.Publisher('SourceSelect', HarkSource)

		append_msg = HarkSourceVal()
		append_msg.id = getSourceDirectionId(self.theta)
		append_msg.theta = self.theta
		msg_select.src.append(append_msg)

		pub_select.publish(msg_select)
		r = rospy.Rate(3)
		r.sleep()
		trigger.publish(True)

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

	global cv_image

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

		global cv_image
		cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")

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
		w = WINDOW_WIDTH
		h = WINDOW_HEIGHT
		return int(w), int(h)

	@property
	def fps(self):
		fps = self._DEFAULT_FPS
		return fps
	
	@QtCore.pyqtSlot()
	def _queryFrame(self):
		global cv_image
		#frame = cv.QueryFrame(self._cameraDevice)
		frame = cv_image

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
	global rightRotateButton
	global leftRotateButton
	global forwardButton
	global backButton

	newFrame = QtCore.pyqtSignal(cv.iplimage)




	def __init__(self, cameraDevice, parent=None):

		super(CameraWidget, self).__init__(parent)
		self._frame = None
		self.frame = QtGui.QFrame(self)
		self.frame.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Raised)
		self.frame.setLineWidth(2)
		self.frame.setGeometry(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT)


		self._cameraDevice = cameraDevice
		self._cameraDevice.newFrame.connect(self._onNewFrame)
		#w, h = self._cameraDevice.frameSize
		#self.setStyleSheet("background-color: white")#背景色を白に
		self.setMinimumSize(WINDOW_WIDTH, WINDOW_HEIGHT)
		self.setMaximumSize(WINDOW_WIDTH, WINDOW_HEIGHT)
#		self.text1 = 'Select'
		self.text2 = u'画面外から話し声がします'


		#ボタンアイコン
		self.auto_left_rotate_button_icon = QtGui.QIcon(self)
		self.auto_left_rotate_button_icon.addFile("/home/user/ros/ui_alt/icon/character_left.png")
		self.auto_right_rotate_button_icon = QtGui.QIcon(self)
		self.auto_right_rotate_button_icon.addFile("/home/user/ros/ui_alt/icon/character_right.png")

		#右矢印1
		self.arrow_right_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x1.png")
		self.arrow_right_x1_image = self.arrow_right_x1_image.scaled(ARROW_WIDTH_X1,ARROW_HEIGHT,0,0)

		#右矢印2
		self.arrow_right_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x2.png")
		self.arrow_right_x2_image = self.arrow_right_x2_image.scaled(ARROW_WIDTH_X2,ARROW_HEIGHT,0,0)

		#右矢印3
		self.arrow_right_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x3.png")
		self.arrow_right_x3_image = self.arrow_right_x3_image.scaled(ARROW_WIDTH_X3,ARROW_HEIGHT,0,0)

		#右矢印1(薄い)
		self.arrow_right_x1_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x1_pale.png")
		self.arrow_right_x1_pale_image = self.arrow_right_x1_pale_image.scaled(ARROW_WIDTH_X1,ARROW_HEIGHT,0,0)

		#右矢印2(薄い)
		self.arrow_right_x2_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x2_pale.png")
		self.arrow_right_x2_pale_image = self.arrow_right_x2_pale_image.scaled(ARROW_WIDTH_X2,ARROW_HEIGHT,0,0)

		#右矢印3(薄い)
		self.arrow_right_x3_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x3_pale.png")
		self.arrow_right_x3_pale_image = self.arrow_right_x3_pale_image.scaled(ARROW_WIDTH_X3,ARROW_HEIGHT,0,0)


		#左矢印1
		self.arrow_left_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x1.png")
		self.arrow_left_x1_image = self.arrow_left_x1_image.scaled(ARROW_WIDTH_X1,ARROW_HEIGHT,0,0)

		#左矢印2
		self.arrow_left_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x2.png")
		self.arrow_left_x2_image = self.arrow_left_x2_image.scaled(ARROW_WIDTH_X2,ARROW_HEIGHT,0,0)

		#左矢印3
		self.arrow_left_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x3.png")
		self.arrow_left_x3_image = self.arrow_left_x3_image.scaled(ARROW_WIDTH_X3,ARROW_HEIGHT,0,0)

		#左矢印1(薄い)
		self.arrow_left_x1_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x1_pale.png")
		self.arrow_left_x1_pale_image = self.arrow_left_x1_pale_image.scaled(ARROW_WIDTH_X1,ARROW_HEIGHT,0,0)

		#左矢印2(薄い)
		self.arrow_left_x2_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x2_pale.png")
		self.arrow_left_x2_pale_image = self.arrow_left_x2_pale_image.scaled(ARROW_WIDTH_X2,ARROW_HEIGHT,0,0)

		#左矢印3(薄い)
		self.arrow_left_x3_pale_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x3_pale.png")
		self.arrow_left_x3_pale_image = self.arrow_left_x3_pale_image.scaled(ARROW_WIDTH_X3,ARROW_HEIGHT,0,0)

		#右向きキャラ
		self.character_right_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_right.png")
		self.character_right_image = self.character_right_image.scaled(ONIMAGE_LISTEN_BUTTON_WIDTH, ONIMAGE_LISTEN_BUTTON_HEIGHT,0,0)


		#左向きキャラ
		self.character_left_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_left.png")
		self.character_left_image = self.character_left_image.scaled(ONIMAGE_LISTEN_BUTTON_WIDTH, ONIMAGE_LISTEN_BUTTON_HEIGHT,0,0)


		#音声視聴ボタン
		self.left_listen_button_icon = QtGui.QIcon(self)
		self.left_listen_button_icon.addFile("/home/user/ros/ui_alt/icon/character_left.png")
		self.right_listen_button_icon = QtGui.QIcon(self)
		self.right_listen_button_icon.addFile("/home/user/ros/ui_alt/icon/character_right.png")

		#左回転ボタン
		self.left_rotate_icon = QtGui.QIcon(self)
		self.left_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		global leftRotateButton
		leftRotateButton = ManualRotateButton(self.left_rotate_icon,"",self)
		leftRotateButton.setButtonCode(LEFT_ROTATE_BUTTON_CODE)
		leftRotateButton.setGeometry(MANUAL_LEFT_ROTATE_BUTTON_XAXIS,MANUAL_LEFT_ROTATE_BUTTON_YAXIS,MANUAL_LEFT_ROTATE_BUTTON_WIDTH,MANUAL_LEFT_ROTATE_BUTTON_HEIGHT)
		leftRotateButton.setIconSize(QtCore.QSize(MANUAL_LEFT_ROTATE_BUTTON_WIDTH,MANUAL_LEFT_ROTATE_BUTTON_HEIGHT))


		#右回転ボタン
		self.right_rotate_icon = QtGui.QIcon(self)
		self.right_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		global rightRotateButton
		rightRotateButton = ManualRotateButton(self.right_rotate_icon,"",self)
		rightRotateButton.setButtonCode(RIGHT_ROTATE_BUTTON_CODE)
		rightRotateButton.setGeometry(MANUAL_RIGHT_ROTATE_BUTTON_XAXIS,MANUAL_RIGHT_ROTATE_BUTTON_YAXIS,MANUAL_RIGHT_ROTATE_BUTTON_WIDTH,MANUAL_RIGHT_ROTATE_BUTTON_HEIGHT)
		rightRotateButton.setIconSize(QtCore.QSize(MANUAL_RIGHT_ROTATE_BUTTON_WIDTH,MANUAL_RIGHT_ROTATE_BUTTON_HEIGHT))


		#前進ボタン
		self.forward_icon = QtGui.QIcon(self)
		self.forward_icon.addFile("/home/user/ros/ui_alt/icon/arrow_forward.png")
		global forwardButton
		forwardButton = ManualRotateButton(self.forward_icon,"",self)
		forwardButton.setButtonCode(FORWARD_BUTTON_CODE)
		forwardButton.setGeometry(MANUAL_FORWARD_XAXIS,MANUAL_FORWARD_YAXIS,MANUAL_FORWARD_WIDTH,MANUAL_FORWARD_HEIGHT)
		forwardButton.setIconSize(QtCore.QSize(MANUAL_FORWARD_WIDTH,MANUAL_FORWARD_HEIGHT))

		#後退ボタン
		self.back_icon = QtGui.QIcon(self)
		self.back_icon.addFile("/home/user/ros/ui_alt/icon/arrow_back.png")
		global backButton
		backButton = ManualRotateButton(self.back_icon,"",self)
		backButton.setButtonCode(BACK_BUTTON_CODE)
		backButton.setGeometry(MANUAL_BACK_XAXIS,MANUAL_BACK_YAXIS,MANUAL_BACK_WIDTH,MANUAL_BACK_HEIGHT)
		backButton.setIconSize(QtCore.QSize(MANUAL_BACK_WIDTH,MANUAL_BACK_HEIGHT))
		
		#分離音声視聴リセットボタン
		self.all_listen_icon = QtGui.QIcon(self)
		self.all_listen_icon.addFile("/home/user/ros/ui_alt/icon/listen.png")
		global allListenButton
		allListenButton = AllListenButton(self.all_listen_icon,"",self)
		allListenButton.setGeometry(ALL_LISTEN_BUTTON_XAXIS,ALL_LISTEN_BUTTON_YAXIS,ALL_LISTEN_BUTTON_WIDTH,ALL_LISTEN_BUTTON_HEIGHT)
		allListenButton.setIconSize(QtCore.QSize(ALL_LISTEN_BUTTON_WIDTH,ALL_LISTEN_BUTTON_HEIGHT))

		#左回転ボタン(自動)
		for i in range(AUTO_ROTATE_BUTTON_NUM):
			#button = AutoRotateButton(self.auto_left_rotate_button_icon,"",self)
			button = AutoRotateButton(self.left_rotate_icon,"",self)
			button.command = LEFT_ROTATE_COMMAND
			leftAutoRotBtnList.append(button)

		#右回転ボタン(自動)
		for i in range(AUTO_ROTATE_BUTTON_NUM):
			#button = AutoRotateButton(self.auto_right_rotate_button_icon,"",self)
			button = AutoRotateButton(self.right_rotate_icon,"",self)
			button.command = RIGHT_ROTATE_COMMAND
			rightAutoRotBtnList.append(button)

		#左分離音声視聴ボタン
		for i in range(LISTEN_BUTTON_NUM):
			button = ListenButton(self.left_listen_button_icon,"",self)
			leftListenBtnList.append(button)

		#右分離音声視聴ボタン
		for i in range(LISTEN_BUTTON_NUM):
			button = ListenButton(self.right_listen_button_icon,"",self)
			rightListenBtnList.append(button)

		#[画面内]左分離音声視聴ボタン
		for i in range(LISTEN_BUTTON_NUM/2):
			button = ListenButton(self.left_listen_button_icon,"",self)
			onImageLeftListenBtnList.append(button)

		#[画面内]右分離音声視聴ボタン
		for i in range(LISTEN_BUTTON_NUM/2):
			button = ListenButton(self.right_listen_button_icon,"",self)
			onImageRightListenBtnList.append(button)



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






	#人の口の近くに音の波を描画
	def paintVoiceWave(self,e):
		global vx
		global vy
		global k1
		global k2
		global k3
		global k4
		global a
		global dflag
		global cflag
		global lflag
		global locSrcList
		global max_power
		

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
		
		if(lflag == True):
			for i in range(len(locSrcList)):
				if(k1 == []):
					break
				print "not break"
				if(math.fabs(k1[0][2] - locSrcList[i].theta) <= 10.0):#音源の位置とkinectでトラッキングした人の位置が近い場合
					power1 = locSrcList[i].power
					path1_1.moveTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 10, k1[1][1] + 10)
					path1_2.moveTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 20, k1[1][1] + 20)
					path1_3.moveTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 30, k1[1][1] + 30)
					path1_4.moveTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 40, k1[1][1] + 40)
					path1_5.moveTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 50, k1[1][1] + 50)
					path1_1.cubicTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 10, k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 10, k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 10, k1[1][1] + 10)
					path1_2.cubicTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 20, k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 20, k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 20, k1[1][1] + 20)
					path1_3.cubicTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 30, k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 30, k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 30, k1[1][1] + 30)
					path1_4.cubicTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 40, k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 40, k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 40, k1[1][1] + 40)
					path1_5.cubicTo(k1[1][0] + CAMERA_IMAGE_OFFSET - 50, k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 50, k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k1[1][0] + CAMERA_IMAGE_OFFSET + 50, k1[1][1] + 50)
			

					if(max_power - power1 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)
						painter3.drawPath(path1_4)
						painter3.drawPath(path1_5)
			
					if(max_power - power1 > 0 and max_power - power1 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)
						painter3.drawPath(path1_4)
   
					if(max_power - power1 > 0.5 and max_power - power1 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)
						painter3.drawPath(path1_3)

					if(max_power - power1 > 1.0 and max_power - power1 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path1_1)
						painter3.drawPath(path1_2)

					if(max_power - power1 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path1_1)
			
			for i in range(len(locSrcList)):

				if(k2 == []):
					break
		
				if(math.fabs(k2[0][2] - locSrcList[i].theta) <= 10.0):
					power2 = locSrcList[i].power
					path2_1.moveTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 10, k2[1][1] + 10)
					path2_2.moveTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 20, k2[1][1] + 20)
					path2_3.moveTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 30, k2[1][1] + 30)
					path2_4.moveTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 40, k2[1][1] + 40)
					path2_5.moveTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 50, k2[1][1] + 50)
					path2_1.cubicTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 10, k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 10, k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 10, k2[1][1] + 10)
					path2_2.cubicTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 20, k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 20, k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 20, k2[1][1] + 20)
					path2_3.cubicTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 30, k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 30, k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 30, k2[1][1] + 30)
					path2_4.cubicTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 40, k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 40, k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 40, k2[1][1] + 40)
					path2_5.cubicTo(k2[1][0] + CAMERA_IMAGE_OFFSET - 50, k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 50, k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k2[1][0] + CAMERA_IMAGE_OFFSET + 50, k2[1][1] + 50)
			

					if(max_power - power2 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)
						painter3.drawPath(path2_4)
						painter3.drawPath(path2_5)
			
					if(max_power - power2 > 0 and max_power - power2 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)
						painter3.drawPath(path2_4)
	   
					if(max_power - power2 > 0.5 and max_power - power2 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)
						painter3.drawPath(path2_3)

					if(max_power - power2 > 1.0 and max_power - power2 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path2_1)
						painter3.drawPath(path2_2)

					if(max_power - power2 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path2_1)	 
			

			for i in range(len(locSrcList)):

				if(k3 == []):
					break

				if(math.fabs(k3[0][2] - locSrcList[i].theta) <= 10.0):
					power3 = locSrcList[i].power
					path3_1.moveTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 10, k3[1][1] + 10)
					path3_2.moveTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 20, k3[1][1] + 20)
					path3_3.moveTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 30, k3[1][1] + 30)
					path3_4.moveTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 40, k3[1][1] + 40)
					path3_5.moveTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 50, k3[1][1] + 50)
					path3_1.cubicTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 10, k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 10, k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 10, k3[1][1] + 10)
					path3_2.cubicTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 20, k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 20, k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 20, k3[1][1] + 20)
					path3_3.cubicTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 30, k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 30, k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 30, k3[1][1] + 30)
					path3_4.cubicTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 40, k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 40, k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 40, k3[1][1] + 40)
					path3_5.cubicTo(k3[1][0] + CAMERA_IMAGE_OFFSET - 50, k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 50, k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k3[1][0] + CAMERA_IMAGE_OFFSET + 50, k3[1][1] + 50)
			

					if(max_power - power3 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)
						painter3.drawPath(path3_4)
						painter3.drawPath(path3_5)
			
					if(max_power - power3 > 0 and max_power - power3 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)
						painter3.drawPath(path3_4)
	   
					if(max_power - power3 > 0.5 and max_power - power3 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)
						painter3.drawPath(path3_3)

					if(max_power - power3 > 1.0 and max_power - power3 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path3_1)
						painter3.drawPath(path3_2)

					if(max_power - power3 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path3_1)
			
			
			for i in range(len(locSrcList)):
			
				if(k4 == []):
					break

				if(math.fabs(k4[0][2] - locSrcList[i].theta) <= 10.0):
			
					power4 = locSrcList[i].power
					path4_1.moveTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 10, k4[1][1] + 10)
					path4_2.moveTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 20, k4[1][1] + 20)
					path4_3.moveTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 30, k4[1][1] + 30)
					path4_4.moveTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 40, k4[1][1] + 40)
					path4_5.moveTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 50, k4[1][1] + 50)
					path4_1.cubicTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 10, k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 10, k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 10, k4[1][1] + 10)
					path4_2.cubicTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 20, k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 20, k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 20, k4[1][1] + 20)
					path4_3.cubicTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 30, k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 30, k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 30, k4[1][1] + 30)
					path4_4.cubicTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 40, k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 40, k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 40, k4[1][1] + 40)
					path4_5.cubicTo(k4[1][0] + CAMERA_IMAGE_OFFSET - 50, k4[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 50, k4[1][1] + 50 +math.sqrt(2 * math.pow(50, 2)), k4[1][0] + CAMERA_IMAGE_OFFSET + 50, k4[1][1] + 50)
			

					if(max_power - power4 == 0):
						painter3.setPen(pen4)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)
						painter3.drawPath(path4_4)
						painter3.drawPath(path4_5)
			
					if(max_power - power4 > 0 and max_power - power4 <= 0.5):
						painter3.setPen(pen3)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)
						painter3.drawPath(path4_4)
	   
					if(max_power - power4 > 0.5 and max_power - power4 <= 1.0):
						painter3.setPen(pen3)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)
						painter3.drawPath(path4_3)

					if(max_power - power4 > 1.0 and max_power - power4 <= 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path4_1)
						painter3.drawPath(path4_2)

					if(max_power - power4 > 1.5):
						painter3.setPen(pen2)
						painter3.drawPath(path4_1)
			
			#直接選択
			for i in range(len(vx)):
				if(dflag == True):
					painter2.setPen(pen4)
					painter2.setFont(QtGui.QFont('Decorative', 14))
					painter2.drawText(vx[i] - 20, 30, self.text1)
	
			#描画処理
			for j in range(len(a)):
				for k in range(1, len(a[j][0])):
					if (cflag == True):
						painter2.setPen(pen5)
						painter2.drawLine(a[j][0][k-1], a[j][1][k-1], a[j][0][k], a[j][1][k])

			#定位情報表示
			if(lflag == True):
				for h in range(len(locSrcList)):
					#画面外情報
					if(locSrcList[h].theta < -30 and locSrcList[h].theta >= -180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220 + CAMERA_IMAGE_OFFSET, 440, self.text2)

					elif(locSrcList[h].theta >= 30 and locSrcList[h].theta < 180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220 + CAMERA_IMAGE_OFFSET, 440, self.text2)


	#音源情報を元に分離音声視聴ボタンを作成
	def showListenButton(self,locSrc):
		global leftListenBtnList
		global rightListenBtnList
		global onImageLeftListenBtnList
		global onImageRightListenBtnList


		#画面内のボタンを表示
		def showOnImageButton(btnList):
			debugTime = time.time()
			#位置が被るボタンが無いか探索
			showButtonFlag = True
			for bindex in range(len(btnList)):
				if btnList[bindex].isVisible():
					if abs(locSrc.x_onImage - btnList[bindex].geometry().left()) < ONIMAGE_SPACE_BETWEEN_AUTO_BUTTON:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						if btnList[bindex].theta < 0:
							btnList[bindex].setGeometry(locSrc.x_onImage + CAMERA_IMAGE_OFFSET - ONIMAGE_LISTEN_BUTTON_WIDTH/2 -60,ONIMAGE_LISTEN_BUTTON_YAXIS,ONIMAGE_LISTEN_BUTTON_WIDTH,ONIMAGE_LISTEN_BUTTON_HEIGHT)
						else:
							btnList[bindex].setGeometry(locSrc.x_onImage + CAMERA_IMAGE_OFFSET - ONIMAGE_LISTEN_BUTTON_WIDTH/2,ONIMAGE_LISTEN_BUTTON_YAXIS,ONIMAGE_LISTEN_BUTTON_WIDTH,ONIMAGE_LISTEN_BUTTON_HEIGHT)
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
					if abs((locSrc.y_arrow + ARROW_HEIGHT/2) -(btnList[bindex].geometry().top() + AUTO_ROTATE_BUTTON_HEIGHT/2)) < SPACE_BETWEEN_AUTO_BUTTON:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						btnList[bindex].setGeometry(xaxis,locSrc.y_arrow + (ARROW_HEIGHT - AUTO_ROTATE_BUTTON_HEIGHT)/2,AUTO_ROTATE_BUTTON_WIDTH,AUTO_ROTATE_BUTTON_HEIGHT)
						btnList[bindex].setVisible(True)
						btnList[bindex].theta = locSrc.theta
						break


		#画面内
		if locSrc.onImageFlag is True:
			if locSrc.theta > 0:
				showOnImageButton(onImageLeftListenBtnList)
			else:
				showOnImageButton(onImageRightListenBtnList)
		#画面外
		else:
			if locSrc.theta > 0:
				showButton(leftListenBtnList,LEFT_LISTEN_BUTTON_XAXIS)
			else:
				showButton(rightListenBtnList,RIGHT_LISTEN_BUTTON_XAXIS)


	#音源情報を元に自動回転ボタンを作成
	def showAutoRotateButton(self,locSrc):
		global leftAutoRotBtnList
		global rightAutoRotBtnList


		#画面外のボタンを表示
		def showButton(btnList,xaxis):
			#位置が被るボタンが無いか探索
			showButtonFlag = True
			for bindex in range(len(btnList)):
				if btnList[bindex].isVisible():
					if abs((locSrc.y_arrow + ARROW_HEIGHT/2) -(btnList[bindex].geometry().top() + AUTO_ROTATE_BUTTON_HEIGHT/2)) < SPACE_BETWEEN_AUTO_BUTTON:
						showButtonFlag = False
						break
			#無ければボタンを描画
			if showButtonFlag:
				for bindex in range(len(btnList)):
					if not btnList[bindex].isVisible():
						btnList[bindex].setGeometry(xaxis,locSrc.y_arrow + (ARROW_HEIGHT - AUTO_ROTATE_BUTTON_HEIGHT)/2,AUTO_ROTATE_BUTTON_WIDTH,AUTO_ROTATE_BUTTON_HEIGHT)
						
						btnList[bindex].setVisible(True)
						btnList[bindex].timeout = abs(TIMEOUT_PER_THETA * locSrc.theta)
						break

		if locSrc.theta > 0:
			showButton(leftAutoRotBtnList,AUTO_LEFT_ROTATE_BUTTON_XAXIS)
		else:
			showButton(rightAutoRotBtnList,AUTO_RIGHT_ROTATE_BUTTON_XAXIS)

	#音源情報を元に分離音声視聴ボタンを削除
	def hideListenButton(self,srcList,vanSrcList):
		debugTime = time.time()
	
		global leftListenBtnList
		global rightListenBtnList
		global onImageLeftListenBtnList
		global onImageRightListenBtnList
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
							if abs((sortSrcList[sindex].x_onImage + CAMERA_IMAGE_OFFSET) -btnList[bindex].geometry().left()) < ONIMAGE_SPACE_BETWEEN_AUTO_BUTTON:
								setInvisibleFlag = False
								break
						else:
							if abs(sortSrcList[sindex].y_arrow -btnList[bindex].geometry().top()) < SPACE_BETWEEN_AUTO_BUTTON:
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
		hideButton(leftSrcList,leftListenBtnList,False)
		hideButton(rightSrcList,rightListenBtnList,False)
		hideButton(onImageLeftSrcList,onImageLeftListenBtnList,True)
		hideButton(onImageRightSrcList,onImageRightListenBtnList,True)


	#自動回転ボタンをかくす
	def hideAutoRotateButton(self,srcList,vanSrcList):
		global leftAutoRotateBtnList
		global rightAutoRotateBtnList
		leftSrcList = []
		rightSrcList = []

		def hideButton(srcList,btnList):
			for bindex in range(len(btnList)):
				setInvisibleFlag = True
				if btnList[bindex].isVisible():
					for sindex in range(len(srcList)):
						if not srcList[sindex].onImageFlag:
							if abs(srcList[sindex].y_arrow -btnList[bindex].geometry().top()) < SPACE_BETWEEN_AUTO_BUTTON:
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
		hideButton(leftSrcList,leftAutoRotBtnList)
		hideButton(rightSrcList,rightAutoRotBtnList)


	#描画処理全般（カメラ画像、情報提示）
	def paintEvent(self, e):

		global vx
		global vy
		global k1
		global k2
		global k3
		global k4
		global a
		global dflag
		global cflag
		global lflag
		global locSrcList
		global vanLocSrcList
		global max_power
		global debugFlag
		


		tmpLocSrcList = locSrcList[:]
		tmpVanLocSrcList= vanLocSrcList[:]

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
			painter1.drawImage(QtCore.QPoint(CAMERA_IMAGE_OFFSET, 0), OpenCVQImage(self._frame))


		#口付近に表示する音の波を描画
		self.paintVoiceWave(e)

		#manualボタンがdisableになっていたら、ableに戻すか判定
		global manualFinDisableTime
		global manualButtonAbleFlag
		if not manualButtonAbleFlag:
			if time.time() > manualFinDisableTime:
				manualButtonAbleFlag = True
				switchManualButtonAble(True)


		#不要ボタンを非表示化
		#自動回転ボタン
		self.hideAutoRotateButton(tmpLocSrcList,tmpVanLocSrcList)
		#左分離音声視聴ボタン
		self.hideListenButton(tmpLocSrcList,tmpVanLocSrcList)


		#ボタン描画
		for index in range(len(tmpLocSrcList)):
			debugTime = time.time()
			if tmpLocSrcList[index].onImageFlag:#画面内
				#分離音声視聴ボタン
				self.showListenButton(tmpLocSrcList[index])

			else:#画面外
				if tmpLocSrcList[index].theta > 0:#左側
					#矢印を描画
					if tmpLocSrcList[index].powerCode == STRONG_POWER_CODE:
						self.arrow_left_x1_point = QtCore.QPoint(ARROW_LEFT_X1_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_left_x1_painter.drawImage(self.arrow_left_x1_point,self.arrow_left_x1_image)
						button_xaxis = AUTO_LEFT_ROTATE_BUTTON_XAXIS
					elif tmpLocSrcList[index].powerCode == MEDIUM_POWER_CODE:
						self.arrow_left_x2_point = QtCore.QPoint(ARROW_LEFT_X2_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_image)
						button_xaxis = AUTO_LEFT_ROTATE_BUTTON_XAXIS
					else:
						self.arrow_left_x3_point = QtCore.QPoint(ARROW_LEFT_X3_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_image)
						button_xaxis = AUTO_LEFT_ROTATE_BUTTON_XAXIS

					#回転ボタン
					self.showAutoRotateButton(tmpLocSrcList[index])

					
					#分離音声視聴ボタン
					self.showListenButton(tmpLocSrcList[index])


				else:#右側
					button_xaxis = 0
					if tmpLocSrcList[index].powerCode == STRONG_POWER_CODE:
						self.arrow_right_x1_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_image)
					elif tmpLocSrcList[index].powerCode == MEDIUM_POWER_CODE:
						self.arrow_right_x2_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_image)
					else:
						self.arrow_right_x3_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpLocSrcList[index].y_arrow)
						arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_image)

					#回転ボタン
					self.showAutoRotateButton(tmpLocSrcList[index])

					#分離音声視聴ボタン
					self.showListenButton(tmpLocSrcList[index])


		#薄矢印描画
		for index in range(len(tmpvanLocSrcList)):
			if not tmpVanLocSrcList[index].onImageFlag:#画面内
				if tmpVanLocSrcList[index].theta > 0:
					if tmpVanLocSrcList[index].powerCode == STRONG_POWER_CODE:
						self.arrow_left_x1_point = QtCore.QPoint(ARROW_LEFT_X1_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x1_painter.drawImage(self.arrow_left_x1_point,self.arrow_left_x1_pale_image)
					elif tmpVanLocSrcList[index].powerCode == MEDIUM_POWER_CODE:
						self.arrow_left_x2_point = QtCore.QPoint(ARROW_LEFT_X2_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_pale_image)
					else:
						self.arrow_left_x3_point = QtCore.QPoint(ARROW_LEFT_X3_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_pale_image)
					
				else:
					if tmpVanLocSrcList[index].powerCode == STRONG_POWER_CODE:
						self.arrow_right_x1_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_pale_image)
					elif tmpVanLocSrcList[index].powerCode == MEDIUM_POWER_CODE:
						self.arrow_right_x2_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_pale_image)
					else:
						self.arrow_right_x3_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,tmpVanLocSrcList[index].y_arrow)
						arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_pale_image)





class MainWindow(QtGui.QMainWindow):
	def __init__(self):
		super(MainWindow, self).__init__()

		#MenuBar
		self.exit_menu = self.menuBar().addMenu("&Exit")
		self.exit = self.exit_menu.addAction(u"終了...", QtGui.QApplication.quit)
		self.exit.setShortcut("Ctrl+C")
		self.exit.setStatusTip(u"アプリケーションを終了します")


		#CameraWidget
		cameraDevice = CameraDevice(mirrored = True)
		self.camerawidget = CameraWidget(cameraDevice)
		self.setCentralWidget(self.camerawidget)
		self.setGeometry(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT)
		#self.setAutoFillBackground(False)
		self.statusBar().showMessage("Welcome to UI-ALT!")
		#self.click.triggered.connect(self.Click)

	def mousePressEvent(self,event):
		p = QtGui.QPixmap.grabWindow(self.winId())
		p.save("scrshot"+str(time.time()),"png")
		
	def keyPressEvent(self,event):
		key = event.key()
		command = None
		if key == Qt.Key_6:
			command = RIGHT_ROTATE_COMMAND
		elif key == Qt.Key_4:
			command = LEFT_ROTATE_COMMAND
		elif key == Qt.Key_8:
			command = FORWARD_COMMAND
		elif key == Qt.Key_2:
			command = BACK_COMMAND
		sendCommand(command,KEY_MANUAL_ROTATE_TIMEOUT)



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

def subscriber():

	rospy.init_node('UIALT', anonymous = True)
	rospy.Subscriber('HarkSource', HarkSource, localization_callback, buff_size = 1)
	rospy.Subscriber('tf_processed1', tf_uialt, tf_callback1)
	rospy.Subscriber('tf_processed2', tf_uialt, tf_callback2)
	rospy.Subscriber('tf_processed3', tf_uialt, tf_callback3)
	rospy.Subscriber('tf_processed4', tf_uialt, tf_callback4)
	#rospy.Subscriber('joy',Joy,joy_callback)
	rospy.spin()


if __name__ == '__main__':
	t = QtThread()
	subscriber()


