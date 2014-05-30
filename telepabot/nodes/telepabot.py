#! /usr/bin/env python
# -*-coding: utf-8 -*-



import roslib; roslib.load_manifest("telepabot")
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
from sensor_msgs.msg import Image# as SIm
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
#from telepabot.msg import tf_telepabot
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
#定位データフォーマットモジュール
import format_loc_src_microcone
#recognition word getter
import recogword
#camera image processer
import cameraimage

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









#central widget
class CentralWidget(QtGui.QWidget):
	#newFrame = QtCore.pyqtSignal(cv.iplimage)

	def __init__(self, cameraDevice, parent=None):

		super(CentralWidget, self).__init__(parent)
		self.cameraImage = None#camera image
		
		#signal timer for slot(paintEvent)
		self.signalTimer = QtCore.QTimer(self)
		self.signalTimer.timeout.connect(self.redraw)
		self.signalTimer.setInterval(const.MSEC_ONE_SEC/const.CENTRAL_WIDGET_FPS)
		self.signalTimer.start()


		#window frame
		self.frame = QtGui.QFrame(self)
		self.frame.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Raised)
		self.frame.setLineWidth(2)
		self.frame.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)

		#self.cameraDevice = cameraDevice
		#self.cameraDevice.iplimageSygnal.connect(self._onNewFrame)
		#self.drawCamImgPoint = const.CAM_IMG_DRAW_POINT
		#w, h = self.cameraDevice.frameSize
		#self.setStyleSheet("background-color: white")#背景色を白に
		
		#set widget size
		self.setMinimumSize(const.WIN_WID, const.WIN_HT)
		self.setMaximumSize(const.WIN_WID, const.WIN_HT)

		#listen range xaxis
		self.listenRangeStartX = 0
		self.listenRangeEndX = 0
		
		#camera painter
		self.cameraPainter = QPainter(self)


	#@QtCore.pyqtSlot()
	def redraw(self):
		self.update()


	def changeEvent(self, e):
		if e.type() == QtCore.QEvent.EnabledChange:
			if self.isEnabled():
				self.cameraDevice.newFrame.connect(self._onNewFrame)
			else:
				self.cameraDevice.newFrame.disconnect(self._onNewFrame)

	#人の口の近くに音の波を描画
	#def paintVoiceWave(self,e):
	#def paintWave(self,point):

	def paintListenRange(self,e):
		if global_var.listenSeparateSoundFlag:
			qp = QtGui.QPainter()
			qp.begin(self)
			color = QtGui.QColor(255, 255, 0, 50)
			qp.setBrush(color)
			qp.drawRect(self.getPaintRect(self.listenRangeStartX,self.listenRangeEndX))

	def paintRecogWord(self,e):
		recogword.adjustWordsPosition()
		recogWordList = global_var.recogWordList
		for horizonList in recogWordList:
			for word in horizonList:
				painter = QtGui.QPainter()
				painter.begin(self)
				painter.drawText(word.boundBox.bottomLeft(),QString(word.text.decode("utf-8")))



	#描画処理全般（カメラ画像、情報提示）
	def paintEvent(self, event):
		#draw recog word
		self.paintRecogWord(event)

		tmpLocSrcList = global_var.locSrcList[:]
		tmpVanLocSrcList= global_var.vanLocSrcList[:]
		
		#have to init in paintEvent
		camImgPainter = QPainter(self)

		cameraimage.drawCameraImage(event,global_var.cvImage,const.CAM_IMG_DRAW_POINT,camImgPainter)

		self.paintListenRange(event)

		#口付近に表示する音の波を描画
		#self.paintVoiceWave(e)


	#クリックが正面映像上でなければX座標を補正する
	def getOnImageX(self,x):
		if x < const.CAM_IMG_OFS_X:
			return const.CAM_IMG_OFS_X
		elif x > (const.CAM_IMG_OFS_X + const.CAM_IMG_WID):
			return const.CAM_IMG_OFS_X + const.CAM_IMG_WID
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
		print "startx:"+str(self.listenRangeStartX - const.CAM_IMG_OFS_X)
		print "endx:" + str(self.listenRangeEndX - const.CAM_IMG_OFS_X)
		if math.fabs(self.listenRangeEndX - self.listenRangeStartX) > const.IGNOR_PIX_THR:
			format_loc_src_microcone.setListenAngles(self.listenRangeStartX - const.CAM_IMG_OFS_X,self.listenRangeEndX - const.CAM_IMG_OFS_X)
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

		#CentralWidget
		cameraDevice = cameraimage.CameraDevice(mirrored = True)
		self.centralWidget = CentralWidget(cameraDevice)
		self.setCentralWidget(self.centralWidget)
		self.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)
		#self.setAutoFillBackground(False)
		self.statusBar().showMessage("Welcome to telepabot!")
		#self.click.triggered.connect(self.Click)



def initialize():
	app = QtGui.QApplication(sys.argv)
	window = MainWindow()
	window.setWindowTitle(const.SYSTEM_NAME)
	window.show()
	recogword.initRecogData()
	#app.exec_()
	return app,window

#トピック購読処理
def subscriber():
	rospy.init_node(const.SYSTEM_NAME, anonymous = True)
	#kinect_tf.subscriber()
	#format_loc_src_microcone.subscriber()
	recogword.subscriber()
	cameraimage.subscriber()
	#rospy.spin()

#メイン関数
if __name__ == '__main__':
	app,window = initialize()
	subscriber()
	app.exec_()
	#sys.exit(app.exec_())
