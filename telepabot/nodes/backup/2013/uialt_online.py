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

#定数(言語仕様では定数は無く、書き換え可能なので注意)
#ウィンドウの大きさ
WINDOW_WIDTH = 1240
WINDOW_HEIGHT = 680

#カメラ映像の大きさと画面内の表示位置
CAMERA_IMAGE_OFFSET = 300
CAMERA_IMAGE_WIDTH = 640
CAMERA_IMAGE_HEIGHT = 480

#右向き矢印の座標
ARROW_RIGHT_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH
ARROW_X1_YAXIS = 40
ARROW_X2_YAXIS = 190
ARROW_X3_YAXIS = 340

#左向き矢印の座標
ARROW_LEFT_XAXIS = 50


#矢印表示の角度閾値
ARROW_X1_THRESHOLD = 28.5
ARROW_X2_THRESHOLD = 50
ARROW_X3_THRESHOLD = 75

#左回転ボタンの位置・大きさ
ARROW_LEFT_ROTATE_XAXIS = 460
ARROW_LEFT_ROTATE_YAXIS = 600
ARROW_LEFT_ROTATE_WIDTH = 50
ARROW_LEFT_ROTATE_HEIGHT = 50

#前進ボタンの位置・大きさ
ARROW_FORWARD_XAXIS = 590
ARROW_FORWARD_YAXIS = 530
ARROW_FORWARD_WIDTH = 50
ARROW_FORWARD_HEIGHT = 50

#右回転ボタンの位置・大きさ
#ARROW_RIGHT_ROTATE_XAXIS = CAMERA_IMAGE_OFFSET + CAMERA_IMAGE_WIDTH + 25
#ARROW_LEFT_ROTATE_XAXIS = CAMERA_IMAGE_OFFSET - 75
#ARROW_ROTATE_X1_YAXIS = ARROW_X1_YAXIS + 30
#ARROW_ROTATE_X2_YAXIS = ARROW_X2_YAXIS + 30
#ARROW_ROTATE_X3_YAXIS = ARROW_X3_YAXIS + 30
ARROW_RIGHT_ROTATE_XAXIS = 720
ARROW_RIGHT_ROTATE_YAXIS = 600
ARROW_RIGHT_ROTATE_WIDTH = 50
ARROW_RIGHT_ROTATE_HEIGHT = 50

#キャラクタアイコンの大きさ
CHARACTER_WIDTH = 80
CHARACTER_HEIGHT = 80

#ボタン識別コード
RIGHT_ROTATE_BUTTON_CODE = 0
LEFT_ROTATE_BUTTON_CODE = 1
FORWARD_BUTTON_CODE = 2


#staticに使いたいインスタンス(今のところコマンド送信用)
LEFT_ROTATE_COMMAND = Twist()
LEFT_ROTATE_COMMAND.angular.z = 0.88
RIGHT_ROTATE_COMMAND = Twist()
RIGHT_ROTATE_COMMAND.angular.z = -0.88
FORWARD_COMMAND = Twist()
FORWARD_COMMAND.linear.x = 0.22
#COMMAND_PUBLISHER = rospy.Publisher('/cmd_vel', Twist)




#グローバル変数
flag = []
dflag = False
cflag = False
lflag = False
sflag = False
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
source = []
msg_select = []
cv_image = None
tfframe = []
prev_msg = HarkSource()

FRAMES = [
	'head',
	'neck',
	'torso',
	'left_shoulder',
	'right_shoulder'
	]

'''
def turtlebot_rcmd():
	
	global source

	cmd = Twist()
	pub = rospy.Publisher('/cmd_vel', Twist)

	cmd.angular.z = -1.0
	os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.1")
	pub.publish(cmd)
	rospy.loginfo(cmd) 
	
def turtlebot_lcmd():

	global source

	cmd = Twist()
	pub = rospy.Publisher('/cmd_vel', Twist)
   
	cmd.angular.z = 0.785
	os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.1")
	pub.publish(cmd)
	rospy.loginfo(cmd)
'''
#画面内の音を選択する用
def src_select():

	global d_theta
	global source
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

#Show localization information
def localization_callback(data):

	global source
	global flagoos
	global max_power
	global prev_msg

	source = []
	flagoos = False

	pub = rospy.Publisher('DetectedSource', HarkSource)
	msg = HarkSource()

	for i in range(len(data.src)):
	
		if(data.src[i].theta >= -45 and data.src[i].theta <= 45):#音源角度から、kinectの画像上のx座標を求める
			x = 320*(1-(math.tan(math.radians(data.src[i].theta))/math.tan(math.radians(28.5))))
		else:#画面外から音が聞こえた場合と思われる
			x = -100
			flagoos = True

		array = [data.src[i].id, x, data.src[i].theta, data.src[i].power]
		source.append(array)

		max_power = source[0][3]
		for j in range(len(source)):
			if source[j][3] - max_power > 0:
				max_power = source[j][3]


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

def tf_callback1(data):
	
	global k1
	k1 = []

	for i in range(len(data.src)):
		k1.append((data.src[i].x, data.src[i].y, data.src[i].theta))
	

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


class Button(QtGui.QPushButton):#長押し対応ボタン
	def __init__(self, *args, **kwargs):
		QtGui.QPushButton.__init__(self, *args, **kwargs)
		self.setAutoRepeat(True)
		self.setAutoRepeatDelay(0)
		#self.setAutoRepeatInterval(1000)
		self.clicked.connect(self.sendCommand)
		self._state = 0
		self.setStyleSheet('QPushButton {color: blue}')

	def setButtonCode(self,code):	#ボタンの種類(引数)毎に送るコマンドを設定
		self.pub = rospy.Publisher('/cmd_vel', Twist)
		self.command = Twist()
		if code == RIGHT_ROTATE_BUTTON_CODE:
			self.command = RIGHT_ROTATE_COMMAND
		elif code == LEFT_ROTATE_BUTTON_CODE:
			self.command = LEFT_ROTATE_COMMAND
		elif code == FORWARD_BUTTON_CODE:
			self.command = FORWARD_COMMAND
		

	def sendCommand(self):#turtlebotに回転・移動コマンドを送信
		if self.isDown():#押している間コマンドを送り続ける
			#if self._state == 0:
			#	self._state = 1
			#	os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.4")
			#	cmd = Twist()
			#	pub = rospy.Publisher('/cmd_vel', Twist)
			#	cmd.angular.z = -0.88
			#	pub.publish(cmd)
			#	rospy.loginfo(cmd)
			#	print 'press'
			#else:
			os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.4")
			self.pub.publish(self.command)
			rospy.loginfo(self.command)
			print 'repeat'
		elif self._state == 1:
			self._state = 0
			os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.0")
			print 'release'
		else:
			print 'click'


	
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
		self.text1 = 'Select'
		self.text2 = u'画面外から話し声がします'
	  
		#右矢印1
		#self.arrow_right_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x1.png")
		#self.arrow_right_x1_image = self.arrow_right_x1_image.scaled(250, 100,0,0)
		#self.arrow_right_x1_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,ARROW_X1_YAXIS)

		#右矢印2
		#self.arrow_right_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x2.png")
		#self.arrow_right_x2_image = self.arrow_right_x2_image.scaled(250, 100,0,0)
		#self.arrow_right_x2_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,ARROW_X2_YAXIS)

		#右矢印3
		self.arrow_right_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_right_x3.png")
		self.arrow_right_x3_image = self.arrow_right_x3_image.scaled(250, 100,0,0)
		self.arrow_right_x3_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,ARROW_X3_YAXIS)

		#左矢印1
		#self.arrow_left_x1_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x1.png")
		#self.arrow_left_x1_image = self.arrow_left_x1_image.scaled(250, 100,0,0)
		#self.arrow_left_x1_point = QtCore.QPoint(ARROW_LEFT_XAXIS,ARROW_X1_YAXIS)

		#左矢印2
		#self.arrow_left_x2_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x2.png")
		#self.arrow_left_x2_image = self.arrow_left_x2_image.scaled(250, 100,0,0)
		#self.arrow_left_x2_point = QtCore.QPoint(ARROW_LEFT_XAXIS,ARROW_X2_YAXIS)

		#左矢印3
		self.arrow_left_x3_image = QtGui.QImage("/home/user/ros/ui_alt/icon/arrow_left_x3.png")
		self.arrow_left_x3_image = self.arrow_left_x3_image.scaled(250, 100,0,0)
		self.arrow_left_x3_point = QtCore.QPoint(ARROW_LEFT_XAXIS,ARROW_X3_YAXIS)
		
		#右向きキャラ
		self.character_right_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_right.png")
		self.character_right_image = self.character_right_image.scaled(CHARACTER_WIDTH, CHARACTER_HEIGHT,0,0)

		#左向きキャラ
		self.character_left_image = QtGui.QImage("/home/user/ros/ui_alt/icon/character_left.png")
		self.character_left_image = self.character_left_image.scaled(CHARACTER_WIDTH, CHARACTER_HEIGHT,0,0)


		"""
		#右回転ボタン(x1)
		self.right_rotate_x1_icon = QtGui.QIcon(self)
		self.right_rotate_x1_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		self.right_rotate_x1_button = Button(self.right_rotate_x1_icon,"",self)
		self.right_rotate_x1_button.setGeometry(ARROW_RIGHT_ROTATE_XAXIS,ARROW_ROTATE_X1_YAXIS,ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT)
		self.right_rotate_x1_button.setIconSize(QtCore.QSize(ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT))
		#self.right_rotate_x1_button.clicked.connect(self.rightRotate)

		#右回転ボタン(x2)
		self.right_rotate_x2_icon = QtGui.QIcon(self)
		self.right_rotate_x2_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		self.right_rotate_x2_button = QtGui.QPushButton(self.right_rotate_x2_icon,"",self)
		self.right_rotate_x2_button.setGeometry(ARROW_RIGHT_ROTATE_XAXIS,ARROW_ROTATE_X2_YAXIS,ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT)
		self.right_rotate_x2_button.setIconSize(QtCore.QSize(ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT))
		self.right_rotate_x2_button.clicked.connect(self.rightRotate)

		#右回転ボタン(x3)
		self.right_rotate_x3_icon = QtGui.QIcon(self)
		self.right_rotate_x3_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		self.right_rotate_x3_button = QtGui.QPushButton(self.right_rotate_x3_icon,"",self)
		self.right_rotate_x3_button.setGeometry(ARROW_RIGHT_ROTATE_XAXIS,ARROW_ROTATE_X3_YAXIS,ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT)
		self.right_rotate_x3_button.setIconSize(QtCore.QSize(ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT))
		self.right_rotate_x3_button.clicked.connect(self.rightRotate)

		#左回転ボタン(x1)
		self.left_rotate_x1_icon = QtGui.QIcon(self)
		self.left_rotate_x1_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		self.left_rotate_x1_button = QtGui.QPushButton(self.left_rotate_x1_icon,"",self)
		self.left_rotate_x1_button.setGeometry(ARROW_LEFT_ROTATE_XAXIS,ARROW_ROTATE_X1_YAXIS,ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT)
		self.left_rotate_x1_button.setIconSize(QtCore.QSize(ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT))
		self.left_rotate_x1_button.clicked.connect(self.leftRotate)

		#左回転ボタン(x2)
		self.left_rotate_x2_icon = QtGui.QIcon(self)
		self.left_rotate_x2_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		self.left_rotate_x2_button = QtGui.QPushButton(self.left_rotate_x2_icon,"",self)
		self.left_rotate_x2_button.setGeometry(ARROW_LEFT_ROTATE_XAXIS,ARROW_ROTATE_X2_YAXIS,ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT)
		self.left_rotate_x2_button.setIconSize(QtCore.QSize(ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT))
		self.left_rotate_x2_button.clicked.connect(self.leftRotate)

		#左回転ボタン(x3)
		self.left_rotate_x3_icon = QtGui.QIcon(self)
		self.left_rotate_x3_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		self.left_rotate_x3_button = QtGui.QPushButton(self.left_rotate_x3_icon,"",self)
		self.left_rotate_x3_button.setGeometry(ARROW_LEFT_ROTATE_XAXIS,ARROW_ROTATE_X3_YAXIS,ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT)
		self.left_rotate_x3_button.setIconSize(QtCore.QSize(ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT))
		self.left_rotate_x3_button.clicked.connect(self.leftRotate)
		"""

		#左回転ボタン
		self.left_rotate_icon = QtGui.QIcon(self)
		self.left_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_left.png")
		self.left_rotate_button = Button(self.left_rotate_icon,"",self)
		self.left_rotate_button.setButtonCode(LEFT_ROTATE_BUTTON_CODE)
		self.left_rotate_button.setGeometry(ARROW_LEFT_ROTATE_XAXIS,ARROW_LEFT_ROTATE_YAXIS,ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT)
		self.left_rotate_button.setIconSize(QtCore.QSize(ARROW_LEFT_ROTATE_WIDTH,ARROW_LEFT_ROTATE_HEIGHT))

				
		#右回転ボタン
		self.right_rotate_icon = QtGui.QIcon(self)
		self.right_rotate_icon.addFile("/home/user/ros/ui_alt/icon/arrow_rotate_right.png")
		self.right_rotate_button = Button(self.right_rotate_icon,"",self)
		self.right_rotate_button.setButtonCode(RIGHT_ROTATE_BUTTON_CODE)
		self.right_rotate_button.setGeometry(ARROW_RIGHT_ROTATE_XAXIS,ARROW_RIGHT_ROTATE_YAXIS,ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT)
		self.right_rotate_button.setIconSize(QtCore.QSize(ARROW_RIGHT_ROTATE_WIDTH,ARROW_RIGHT_ROTATE_HEIGHT))



				
		#前進ボタン
		self.forward_icon = QtGui.QIcon(self)
		self.forward_icon.addFile("/home/user/ros/ui_alt/icon/arrow_forward.png")
		self.forward_button = Button(self.forward_icon,"",self)
		self.forward_button.setButtonCode(FORWARD_BUTTON_CODE)
		self.forward_button.setGeometry(ARROW_FORWARD_XAXIS,ARROW_FORWARD_YAXIS,ARROW_FORWARD_WIDTH,ARROW_FORWARD_HEIGHT)
		self.forward_button.setIconSize(QtCore.QSize(ARROW_LEFT_ROTATE_WIDTH,ARROW_FORWARD_HEIGHT))




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
		global source
		global max_power


		if self._frame is None:
			return

		painter1 = QtGui.QPainter(self)
		painter2 = QtGui.QPainter(self)#直接選択時の「SELECT」用
		painter3 = QtGui.QPainter(self)
		painter4 = QtGui.QPainter(self)#定位情報表示用
		painter5 = QtGui.QPainter(self)

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
		painter1.drawImage(QtCore.QPoint(CAMERA_IMAGE_OFFSET, 0), OpenCVQImage(self._frame))
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
			for i in range(len(source)):
				if(k1 == []):
					break
				if(math.fabs(k1[0][2] - source[i][2]) <= 10.0):
					power1 = source[i][3]
					path1_1.moveTo(k1[1][0] - 10, k1[1][1] + 10)
					path1_2.moveTo(k1[1][0] - 20, k1[1][1] + 20)
					path1_3.moveTo(k1[1][0] - 30, k1[1][1] + 30)
					path1_4.moveTo(k1[1][0] - 40, k1[1][1] + 40)
					path1_5.moveTo(k1[1][0] - 50, k1[1][1] + 50)
					path1_1.cubicTo(k1[1][0] - 10, k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k1[1][0] + 10, k1[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k1[1][0] + 10, k1[1][1] + 10)
					path1_2.cubicTo(k1[1][0] - 20, k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k1[1][0] + 20, k1[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k1[1][0] + 20, k1[1][1] + 20)
					path1_3.cubicTo(k1[1][0] - 30, k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k1[1][0] + 30, k1[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k1[1][0] + 30, k1[1][1] + 30)
					path1_4.cubicTo(k1[1][0] - 40, k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k1[1][0] + 40, k1[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k1[1][0] + 40, k1[1][1] + 40)
					path1_5.cubicTo(k1[1][0] - 50, k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k1[1][0] + 50, k1[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k1[1][0] + 50, k1[1][1] + 50)
			

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
			
			for i in range(len(source)):

				if(k2 == []):
					break
		
				if(math.fabs(k2[0][2] - source[i][2]) <= 10.0):
					power2 = source[i][3]
					path2_1.moveTo(k2[1][0] - 10, k2[1][1] + 10)
					path2_2.moveTo(k2[1][0] - 20, k2[1][1] + 20)
					path2_3.moveTo(k2[1][0] - 30, k2[1][1] + 30)
					path2_4.moveTo(k2[1][0] - 40, k2[1][1] + 40)
					path2_5.moveTo(k2[1][0] - 50, k2[1][1] + 50)
					path2_1.cubicTo(k2[1][0] - 10, k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k2[1][0] + 10, k2[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k2[1][0] + 10, k2[1][1] + 10)
					path2_2.cubicTo(k2[1][0] - 20, k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k2[1][0] + 20, k2[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k2[1][0] + 20, k2[1][1] + 20)
					path2_3.cubicTo(k2[1][0] - 30, k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k2[1][0] + 30, k2[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k2[1][0] + 30, k2[1][1] + 30)
					path2_4.cubicTo(k2[1][0] - 40, k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k2[1][0] + 40, k2[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k2[1][0] + 40, k2[1][1] + 40)
					path2_5.cubicTo(k2[1][0] - 50, k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k2[1][0] + 50, k2[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k2[1][0] + 50, k2[1][1] + 50)
			

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
			

			for i in range(len(source)):

				if(k3 == []):
					break

				if(math.fabs(k3[0][2] - source[i][2]) <= 10.0):
					power3 = source[i][3]
					path3_1.moveTo(k3[1][0] - 10, k3[1][1] + 10)
					path3_2.moveTo(k3[1][0] - 20, k3[1][1] + 20)
					path3_3.moveTo(k3[1][0] - 30, k3[1][1] + 30)
					path3_4.moveTo(k3[1][0] - 40, k3[1][1] + 40)
					path3_5.moveTo(k3[1][0] - 50, k3[1][1] + 50)
					path3_1.cubicTo(k3[1][0] - 10, k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k3[1][0] + 10, k3[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k3[1][0] + 10, k3[1][1] + 10)
					path3_2.cubicTo(k3[1][0] - 20, k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k3[1][0] + 20, k3[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k3[1][0] + 20, k3[1][1] + 20)
					path3_3.cubicTo(k3[1][0] - 30, k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k3[1][0] + 30, k3[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k3[1][0] + 30, k3[1][1] + 30)
					path3_4.cubicTo(k3[1][0] - 40, k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k3[1][0] + 40, k3[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k3[1][0] + 40, k3[1][1] + 40)
					path3_5.cubicTo(k3[1][0] - 50, k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k3[1][0] + 50, k3[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k3[1][0] + 50, k3[1][1] + 50)
			

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
			
			
			for i in range(len(source)):
			
				if(k4 == []):
					break

				if(math.fabs(k4[0][2] - source[i][2]) <= 10.0):
			
					power4 = source[i][3]
					path4_1.moveTo(k4[1][0] - 10, k4[1][1] + 10)
					path4_2.moveTo(k4[1][0] - 20, k4[1][1] + 20)
					path4_3.moveTo(k4[1][0] - 30, k4[1][1] + 30)
					path4_4.moveTo(k4[1][0] - 40, k4[1][1] + 40)
					path4_5.moveTo(k4[1][0] - 50, k4[1][1] + 50)
					path4_1.cubicTo(k4[1][0] - 10, k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k4[1][0] + 10, k4[1][1] + 10 + math.sqrt(2 * math.pow(10, 2)), k4[1][0] + 10, k4[1][1] + 10)
					path4_2.cubicTo(k4[1][0] - 20, k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k4[1][0] + 20, k4[1][1] + 20 + math.sqrt(2 * math.pow(20, 2)), k4[1][0] + 20, k4[1][1] + 20)
					path4_3.cubicTo(k4[1][0] - 30, k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k4[1][0] + 30, k4[1][1] + 30 + math.sqrt(2 * math.pow(30, 2)), k4[1][0] + 30, k4[1][1] + 30)
					path4_4.cubicTo(k4[1][0] - 40, k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k4[1][0] + 40, k4[1][1] + 40 + math.sqrt(2 * math.pow(40, 2)), k4[1][0] + 40, k4[1][1] + 40)
					path4_5.cubicTo(k4[1][0] - 50, k4[1][1] + 50 + math.sqrt(2 * math.pow(50, 2)), k4[1][0] + 50, k4[1][1] + 50 +math.sqrt(2 * math.pow(50, 2)), k4[1][0] + 50, k4[1][1] + 50)
			

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
				for h in range(len(source)):
					#画面外情報
					if(source[h][2] < -30 and source[h][2] >= -180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220, 440, self.text2)

					elif(source[h][2] >= 30 and source[h][2] < 180):
						painter4.setPen(pen1)
						painter4.setFont(QtGui.QFont('Helvetica', 16))
						painter4.drawText(220, 440, self.text2)



		#位置テスト用
		#arrow_left_x1_painter.drawImage(self.arrow_left_x1_point,self.arrow_left_x1_image)
		#arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_image)
		#arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_image)
		#arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_image)
		#arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_image)
		#arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_image)

		#矢印描画
		for index in range(len(source)):
			if(source[index][2] > ARROW_X1_THRESHOLD):# and source[h][2] <= ARROW_X2_THRESHOLD
				tmp = int((source[index][2]-ARROW_X1_THRESHOLD)/5)#5度単位で表示位置を変えるため、5n度のnを求める				
				self.arrow_left_x3_point = QtCore.QPoint(ARROW_LEFT_XAXIS,tmp*20)
				arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_image)
			elif(source[index][2] <= -ARROW_X1_THRESHOLD):
				tmp = int((source[index][2]+ARROW_X1_THRESHOLD)/5)#5度単位で表示位置を変えるため、5n度のnを求める
				self.arrow_right_x3_point = QtCore.QPoint(ARROW_RIGHT_XAXIS,-tmp*20)
				arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_image)
			elif(ARROW_X1_THRESHOLD > source[index][2] and source[index][2] > -ARROW_X1_THRESHOLD):
				if source[index][1] >= 320:
					self.character_right_point = QtCore.QPoint(source[index][1] + CAMERA_IMAGE_OFFSET - CHARACTER_WIDTH / 2,50)
					character_right_painter.drawImage(self.character_right_point,self.character_right_image)					
				else:
					self.character_left_point = QtCore.QPoint(source[index][1] + CAMERA_IMAGE_OFFSET - CHARACTER_WIDTH / 2 ,50)
					character_left_painter.drawImage(self.character_left_point,self.character_left_image)


			#elif(source[h][2] > ARROW_X2_THRESHOLD and source[h][2] <= ARROW_X3_THRESHOLD):
				#arrow_left_x2_painter.drawImage(self.arrow_left_x2_point,self.arrow_left_x2_image)
			#elif(source[h][2] > ARROW_X3_THRESHOLD):
				#arrow_left_x3_painter.drawImage(self.arrow_left_x3_point,self.arrow_left_x3_image)
			#elif(source[h][2] > -ARROW_X2_THRESHOLD and source[h][2] <= -ARROW_X1_THRESHOLD):
				#arrow_right_x1_painter.drawImage(self.arrow_right_x1_point,self.arrow_right_x1_image)
			#elif(source[h][2] > -ARROW_X3_THRESHOLD and source[h][2] <= -ARROW_X2_THRESHOLD):
				#arrow_right_x2_painter.drawImage(self.arrow_right_x2_point,self.arrow_right_x2_image)
			#elif(source[h][2] <= -ARROW_X3_THRESHOLD):
				#arrow_right_x3_painter.drawImage(self.arrow_right_x3_point,self.arrow_right_x3_image)


	def mousePressEvent(self, e):

		global vx
		global vy
		global xarray
		global yarray
		global a
		global max_theta
		global min_theta
		global d_theta
		global id
		global dfrag
		global cflag
		global sflag
		global flagoos

		array = [xarray, yarray]

		#直接選択
		if(e.button() == 1 and dflag == True):
			vx.append(e.x())
			vy.append(e.y())
			id.append(len(vx))
	
			if(e.x() <= 320):
				d_theta.append(math.degrees(math.atan(math.fabs(e.x() - 320) / 320 * math.tan(math.radians(28.5)))))
			else: 
				d_theta.append(-(math.degrees(math.atan(math.fabs(e.x() - 320) /320 * math.tan(math.radians(28.5))))))
			sflag = True
			src_select()
		
		#描画選択
		if(e.button() == 1 and cflag == True):

			xarray.append(e.x())
			yarray.append(e.y())
			a.append(array)

		#Reset
		if(e.button() == 2):
		
			vx = []
			vy = []
			a = []
			max_theta = []
			min_theta = []
			d_theta = []
			id = []
			sflag = False
			print("reset")
			src_select()

	def mouseMoveEvent(self, e):
	
		global xarray
		global yarray
		global cflag

		if(cflag == True and xarray != [] and yarray != []):
			xarray.append(e.x())
			yarray.append(e.y())
	
	def mouseReleaseEvent(self, e):

		global max_theta
		global min_theta
		global xarray
		global yarray
		global cflag
		global sflag

		if(e.button() == 1 and cflag == True):
			xarray.append(e.x()) 
			yarray.append(e.y())
			max_x = max(xarray)
			min_x = min(xarray)

			#max_theta
			if(max_x >= 320):
				max_theta.append(math.degrees(math.atan(math.fabs(max_x-320)/320*math.tan(math.radians(28.5)))))
			else:
				max_theta.append(-(math.degrees(math.atan(math.fabs(max_x-320)/320*math.tan(math.radians(28.5))))))

			#min_theta
			if(min_x <= 320):
				min_theta.append(-(math.degrees(math.atan(math.fabs(min_x-320)/320*math.tan(math.radians(28.5))))))
			else:
				min_theta.append(math.degrees(math.atan(math.fabs(min_x-320)/320*math.tan(math.radians(28.5)))))

			xarray = []
			yarray = []
			sflag = True
			src_select()

class MainWindow(QtGui.QMainWindow):
	def __init__(self):
		super(MainWindow, self).__init__()

		#MenuBar
		self.exit_menu = self.menuBar().addMenu("&Exit")
		self.exit = self.exit_menu.addAction(u"終了...", QtGui.QApplication.quit)
		self.exit.setShortcut("Ctrl+C")
		self.exit.setStatusTip(u"アプリケーションを終了します")

		self.mode_menu = self.menuBar().addMenu("&Select Mode")
		self.click = QtGui.QAction(u"クリック", self, checkable = True)
		self.click.setStatusTip(u"UI画面上でマウスクリックで聴きたい音源を直接選択できます")
		self.click.triggered.connect(self.Click)
		self.mode_menu.addAction(self.click)
		self.draw = QtGui.QAction(u"描画", self, checkable = True)
		self.draw.setStatusTip(u"UI画面上でマウスで線を描くことにより音源を選択できます")
		self.draw.triggered.connect(self.Draw)
		self.mode_menu.addAction(self.draw)
		self.slider = QtGui.QAction(u"スライダー", self, checkable = True)
		self.slider.setStatusTip(u"スライダーで聴きたい音源範囲を選択できます")
		self.slider.triggered.connect(self.Slider)
		self.mode_menu.addAction(self.slider)

		self.i_menu = self.menuBar().addMenu("&Information")
		self.information = QtGui.QAction(u"話者情報", self, checkable = True)
		self.information.setStatusTip(u"UI画面上に話者の情報を提示します")
		self.information.triggered.connect(self.Information)
		self.i_menu.addAction(self.information)

		#CameraWidget
		cameraDevice = CameraDevice(mirrored = True)
		self.camerawidget = CameraWidget(cameraDevice)
		self.setCentralWidget(self.camerawidget)
		self.setGeometry(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT)
		#self.setAutoFillBackground(False)
		self.statusBar().showMessage("Welcome to UI-ALT!")

	def Click(self):

		global dflag
		f = 0

		if(dflag == False and f == 0):
			dflag = True
			f = 1
			print('direct: ON')

		if(dflag == True and f == 0):
			dflag = False
			f = 1
			print('direct: OFF')

	def Draw(self):

		global cflag
		f = 0

		if(cflag == False and f == 0):
			cflag = True
			f = 1
			print('circle: ON')

		if(cflag == True and f == 0):
			cflag = False
			f = 1
			print('circle: OFF')

	def Slider(self):
		print("slider")

	def Information(self):

		global lflag
		f = 0

		if(lflag == False and f == 0):
			lflag = True
			f = 1
			print('localization: ON')

		if(lflag == True and f == 0):
			lflag = False
			f = 1
			print('localization: OFF')

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
	rospy.spin()

if __name__ == '__main__':
	t = QtThread()
	subscriber()
