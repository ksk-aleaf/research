#! /usr/bin/env python
# -*-coding: utf-8 -*-



import roslib; roslib.load_manifest("telepabot")
import rospy
import copy

#必要なメッセージファイル

from geometry_msgs.msg import Twist
#from sensor_msgs.msg import Joy

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import math

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
#csv log module
import csvlog



#turtlebotへコマンドを送信
def sendCommand(command):
	#os.system(const.SET_TO_STR + str(timeout))
	#pub = rospy.Publisher('/cmd_vel', Twist)
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist)
	pub.publish(command)
	rospy.loginfo(command)


#central widget
class CentralWidget(QtGui.QWidget):
	def __init__(self, parent=None):

		super(CentralWidget, self).__init__(parent)
		self.signalTimer = QtCore.QTimer(self)
		self.signalTimer.timeout.connect(self.redraw)
		self.signalTimer.setInterval(const.MSEC_ONE_SEC/const.CENTRAL_WIDGET_FPS)
		self.signalTimer.start()


		#window frame
		self.frame = QtGui.QFrame(self)
		self.frame.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Raised)
		self.frame.setLineWidth(2)
		self.frame.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)

		
		#set widget size
		self.setMinimumSize(const.WIN_WID, const.WIN_HT)
		self.setMaximumSize(const.WIN_WID, const.WIN_HT)

		#listen range xaxis
		self.listenRangeStartX = 0
		self.listenRangeEndX = 0
		
		#camera painter
		self.cameraPainter = QPainter(self)
		
		#loc src painter
		self.locSrcPainter = QPainter(self)
		self.locSrcPainter.setFont(QFont('Decorative',14))
		


	#@QtCore.pyqtSlot()
	def redraw(self):
		self.update()


	def changeEvent(self, e):
		if e.type() == QtCore.QEvent.EnabledChange:
			if self.isEnabled():
				self.cameraDevice.newFrame.connect(self._onNewFrame)
			else:
				self.cameraDevice.newFrame.disconnect(self._onNewFrame)


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

	def paintLocVoice(self):
		tmpLocSrcList = copy.deepcopy(global_var.locSrcList)
		for locSrc in tmpLocSrcList:
			painter = QtGui.QPainter()
			painter.begin(self)
			painter.setPen(QPen(QColor(255,0,0,255)))
			painter.setFont(QFont('Decorative',14))
			painter.drawText(locSrc.drawBottomLeft,QString(const.LOC_STR.decode("utf-8")))


	def paintCamImg(self,event):
		#have to init in paintEvent
		centerCamImgPainter = QPainter(self)
		leftCamImgPainter = QPainter(self)
		rightCamImgPainter = QPainter(self)

		if global_var.cvCenterImage is not None:
			cameraimage.drawCameraImage(event,global_var.cvCenterImage,QtGui.QImage.Format_RGB888,const.CENTER_CAM_IMG_DRAW_POINT,centerCamImgPainter)
		if global_var.cvLeftImage is not None:
			cameraimage.drawCameraImage(event,global_var.cvLeftImage,QtGui.QImage.Format_RGB888,const.LEFT_CAM_IMG_DRAW_POINT,leftCamImgPainter)
		if global_var.cvRightImage is not None:
			cameraimage.drawCameraImage(event,global_var.cvRightImage,QtGui.QImage.Format_RGB888,const.RIGHT_CAM_IMG_DRAW_POINT,rightCamImgPainter)

	#描画処理全般（カメラ画像、情報提示）
	def paintEvent(self, event):
		#draw recog word
		self.paintRecogWord(event)
		self.paintLocVoice()
		#tmpLocSrcList = global_var.locSrcList[:]
		#tmpVanLocSrcList= global_var.vanLocSrcList[:]

		self.paintCamImg(event)
		self.paintListenRange(event)


	def mousePressEvent(self,event):
		self.listenRangeStartX = event.x()
		self.listenRangeEndX =  self.listenRangeStartX
		global_var.listenSeparateSoundFlag = True
		#p = QtGui.QPixmap.grabWindow(self.winId())
		#p.save("scrshot"+str(time.time()),"png")

	def mouseDoubleClickEvent(self,event):
		format_loc_src_microcone.listenWholeSound()
		global_var.listenSeparateSoundFlag = False

	def mouseMoveEvent(self,event):
		self.listenRangeEndX = event.x()

	def mouseReleaseEvent(self, e):
		#座標を角度に変換してグローバル変数にセット
		print "startx:"+str(self.listenRangeStartX - const.CAM_IMG_OFS_X)
		print "endx:" + str(self.listenRangeEndX - const.CAM_IMG_OFS_X)
		if math.fabs(self.listenRangeEndX - self.listenRangeStartX) > const.IGNOR_PIX_THR:
			format_loc_src_microcone.setListenAngles(self.listenRangeStartX,self.listenRangeEndX)
			format_loc_src_microcone.listenSeparateSound()
			global_var.listenSeparateSoundFlag = True
		else:
			self.listenRangeEndX = self.listenRangeStartX
			global_var.listenSeparateSoundFlag = False
			format_loc_src_microcone.listenWholeSound()
		print "separate"
		print "from(onUI):" + str(global_var.listenRangeStartAngle)
		print "to(onUI):" + str(global_var.listenRangeEndAngle)





	def getPaintRect(self,startX,endX):
		absRange = math.fabs(endX - startX)
		
		if startX < endX:
			return QRect(startX,const.CAM_IMG_OFS_Y,absRange,const.CAM_IMG_HT)
		else:
			return QRect(endX,const.CAM_IMG_OFS_Y,absRange,const.CAM_IMG_HT)


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
		self.centralWidget = CentralWidget()
		self.setCentralWidget(self.centralWidget)
		self.setGeometry(0, 0, const.WIN_WID, const.WIN_HT)
		#self.setAutoFillBackground(False)
		self.statusBar().showMessage("Welcome to telepabot!")
		#self.click.triggered.connect(self.Click)
	
	def keyPressEvent(self,event):
		print "keyPressed"
		key = event.key()
		command = None
		if key == Qt.Key_Right:
			print "key_6_Pressed"
			command = const.R_ROT_CMD
		elif key == Qt.Key_Left:
			print "key_4_Pressed"
			command = const.L_ROT_CMD
		elif key == Qt.Key_Up:
			print "key_8_Pressed"
			command = const.FWD_CMD
		elif key == Qt.Key_Down:
			print "key_2_Pressed"
			command = const.BACK_CMD
		#sendCommand(command,const.KEY_MAN_ROT_TO)
		if command is not None:
			sendCommand(command)

def signal_handler(signal, frame):
	print('You pressed Ctrl+C!')
	csvlog.close()
	sys.exit(0)


def initialize():
	app = QtGui.QApplication(sys.argv)
	window = MainWindow()
	window.setWindowTitle(const.SYSTEM_NAME)
	window.show()
	recogword.initRecogData()
	#signal.signal(signal.SIGINT, signal_handler)
	return app,window

#トピック購読処理
def subscriber():
	rospy.init_node(const.SYSTEM_NAME, anonymous = True)
	#kinect_tf.subscriber()
	format_loc_src_microcone.subscriber()
	recogword.subscriber()
	cameraimage.subscriber()
	#rospy.spin()

#メイン関数
if __name__ == '__main__':
	subscriber()
	#need to get window
	app,window = initialize()

	#do this method last in main(system loop start)
	app.exec_()
	#sys.exit(app.exec_())
