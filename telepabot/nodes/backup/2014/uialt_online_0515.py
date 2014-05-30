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
from hark_msgs.msg import HarkJuliusSrc
from hark_msgs.msg import HarkJuliusSrcVal
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
#~ import button
#~ from button import AllListenButton
#~ from button import ManualRotateButton
#~ from button import AutoRotateButton
#~ from button import ListenButton
#kinect tracking file モジュール
#import kinect_tf
#定位データフォーマットモジュール
import format_loc_src_microcone

#~ FRAMES = [
	#~ 'head',
	#~ 'neck',
	#~ 'torso',
	#~ 'left_shoulder',
	#~ 'right_shoulder'
	#~ ]


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
	
	#ros::param::set("/usb_cam/pixel_format", "yuyv");

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
		self.im_sub = rospy.Subscriber("/usb_cam/image_raw", SIm, self.image_callback)
		#self.im_sub = rospy.Subscriber("/camera/rgb/image_color_decompressed", SIm, self.image_callback)
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
		#self.text1 = 'Select'
		self.text2 = u'画面外から話し声がします'

		#視聴範囲
		self.listenRangeStartX = 0
		self.listenRangeEndX = 0



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
	#def paintVoiceWave(self,e):
	#def paintWave(self,point):

	def paintListenRange(self):
		if global_var.listenSeparateSoundFlag:
			qp = QtGui.QPainter()
			qp.begin(self)
			color = QtGui.QColor(255, 255, 0, 50)
			qp.setBrush(color)
			qp.drawRect(self.getPaintRect(self.listenRangeStartX,self.listenRangeEndX))

	#描画処理全般（カメラ画像、情報提示）
	def paintEvent(self, e):
		painter = QPainter(self)
		pen = QPen(QColor.green)
		painter.setPen(pen)
		painter.setFont(QFont('Decorative',14))
		painter.drawText(10,10,"testDraw")

		tmpLocSrcList = global_var.locSrcList[:]
		tmpVanLocSrcList= global_var.vanLocSrcList[:]

		painter1 = QtGui.QPainter(self)

		#カメライメージ描画
		if self._frame is not None:
			painter1.drawImage(QtCore.QPoint(const.CAM_IMG_OFS, 0), OpenCVQImage(self._frame))

		self.paintListenRange()

		#口付近に表示する音の波を描画
		#self.paintVoiceWave(e)


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
		window.setWindowTitle(SYSTEM_NAME)
		window.show()
		app.exec_()

	t = threading.Thread(None, main, 'Qt')
	t.setDaemon(1)
	t.start()

	#Subscriber 



#トピック購読処理
def subscriber():
	rospy.init_node(SYSTEM_NAME, anonymous = True)
	#kinect_tf.subscriber()
	format_loc_src_microcone.subscriber()
	rospy.spin()

#メイン関数
if __name__ == '__main__':
	t = QtThread()
	subscriber()
