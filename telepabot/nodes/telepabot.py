#! /usr/bin/env python
# -*-coding: utf-8 -*-



import roslib; roslib.load_manifest("telepabot")
import rospy
import copy

#必要なメッセージファイル


#from sensor_msgs.msg import Joy

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import math
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
#csv log module
import csvlog

import thetaimg

#manipulate robot module
import manipulate_turtlebot2
import time
from geometry_msgs.msg import Twist

# class globalVar():
# 	def __init__(self):
# 	self.

#第一引数より第二引数が大きければ入れ替える
def sortNums(var1,var2):
	if var1 > var2:
		return var2,var1
	else:
		return var1,var2

#pointがstart,endの間にあるか判定
def ifNumBetween(start,end,point):
	if start <= point and point <= end:
		return True
	else:
		return False

#turtlebotへコマンドを送信
def sendCommand(command):
	#os.system(const.SET_TO_STR + str(timeout))
	#pub = rospy.Publisher('/cmd_vel', Twist)
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist)
	pub.publish(command)
	rospy.loginfo(command)


#分離音声視聴範囲クラス
class CrosswideRange():
	def __init__(self,startX = 0,endX = 0,startAzimuth = 0,endAzimuth = 0):
		self.startX,self.endX,self.startAzimuth,self.endAzimuth = startX,endX,startAzimuth,endAzimuth
	def printRangeInfo(self):
		print "startX:"+str(self.startX)+"  endX:"+str(self.endX)

#分離音声視聴リスト初期化
def initListenRange():
	global_var.listenRangeList = []
	for index in range(const.LISTEN_SEPARATE_SOUND_MAX_NUM):
		global_var.listenRangeList.append(CrosswideRange())


#ウィンドウ定義クラス
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

		
		#camera painter
		self.cameraPainter = QPainter(self)
		
		#loc src painter
		self.locSrcPainter = QPainter(self)
		self.locSrcPainter.setFont(QFont('Decorative',14))

		#Input Status
		self.ifDragging = False
		self.ifDoubleClicked = False

	#@QtCore.pyqtSlot()
	def redraw(self):
		self.update()


	def changeEvent(self, e):
		if e.type() == QtCore.QEvent.EnabledChange:
			if self.isEnabled():
				self.cameraDevice.newFrame.connect(self._onNewFrame)
			else:
				self.cameraDevice.newFrame.disconnect(self._onNewFrame)

	#色情報を取得
	#color:色 alpha:透過度
	def getColor(self,colorStr,alpha):
		color = QtGui.QColor()
		color.setNamedColor(colorStr)
		color.setAlpha(alpha)
		return color

	#聴取範囲を正面(constのLEFT~RIGHT_FRONT_AREA)と左右に分離
	def getDrawRanges(self,listenRange):
		frontListenRange = None
		leftSideListenRange = None
		rightSideListenRange = None
		
		if listenRange.endX < const.LEFT_FRONT_AREA_X:
			leftSideListenRange = listenRange

		elif listenRange.startX > const.RIGHT_FRONT_AREA_X:
			rightSideListenRange = listenRange

		elif listenRange.startX < const.LEFT_FRONT_AREA_X and listenRange.endX > const.LEFT_FRONT_AREA_X:
			leftSideListenRange = CrosswideRange(listenRange.startX,const.LEFT_FRONT_AREA_X)
			
			if listenRange.endX < const.RIGHT_FRONT_AREA_X:
				frontListenRange = CrosswideRange(const.LEFT_FRONT_AREA_X,listenRange.endX)
			else:
				frontListenRange = CrosswideRange(const.LEFT_FRONT_AREA_X,const.RIGHT_FRONT_AREA_X)
				rightSideListenRange = CrosswideRange(const.RIGHT_FRONT_AREA_X,listenRange.endX)

		elif listenRange.startX >= const.LEFT_FRONT_AREA_X:

			if listenRange.endX < const.RIGHT_FRONT_AREA_X:
				frontListenRange = listenRange
			else:
				frontListenRange = CrosswideRange(listenRange.startX,const.RIGHT_FRONT_AREA_X)
				rightSideListenRange = CrosswideRange(const.RIGHT_FRONT_AREA_X,listenRange.endX)
		
		return leftSideListenRange,frontListenRange,rightSideListenRange#,darkFilterRangeList
		

	#選択聴取範囲(単体)を描画
	def paintListenRange(self,event,listenRange,listenRangeIndex):
		
		#startXの方が小さくなるようソート
		startX,endX = sortNums(listenRange.startX, listenRange.endX)
		
		#メイン視聴範囲、サブ視聴範囲で色を分ける
		if listenRangeIndex == const.MAIN_RANGE_INDEX:
			rangeDrawColor = const.MAIN_RANGE_DRAW_COLOR_STR
		else:
			rangeDrawColor = const.SUB_RANGE_DRAW_COLOR_STR

		#正面、それ以外で明るさを分ける
		leftSideListenRange,frontListenRange,rightSideListenRange = self.getDrawRanges(CrosswideRange(startX,endX))
		self.paintRect(event,self.getColor(rangeDrawColor,const.RANGE_DRAW_FRONT_ALPHA),frontListenRange)
		self.paintRect(event,self.getColor(rangeDrawColor,const.RANGE_DRAW_SIDE_ALPHA),leftSideListenRange)
		self.paintRect(event,self.getColor(rangeDrawColor,const.RANGE_DRAW_SIDE_ALPHA),rightSideListenRange)


	#選択聴取範囲(複数)を描画
	def paintListenRanges(self,event):
		for index in range(global_var.listenSeparateSoundCount):
			self.paintListenRange(event,copy.deepcopy(global_var.listenRangeList[index]),index)

	#暗くするフィルタを描画するか判断(最低1つの選択範囲が選択完了されている時のみ)
	def ifPaintDarkFilter(self):
		if global_var.listenSeparateSoundCount == const.LISTEN_SEPARATE_SOUND_MAX_NUM or (global_var.listenSeparateSoundCount > 0 and self.ifDragging is False):
			return True
		else:
			return False

	#正面以外、かつ選択聴取範囲外を暗くする
	def paintDarkFilter(self,event):

		if self.ifPaintDarkFilter() is True:#listenSeparateSoundCount > 0 を保証
			rangeList = []
			
			#視聴範囲を取得
			for index in range(global_var.listenSeparateSoundCount):
				rangeList.append(copy.deepcopy(global_var.listenRangeList[index]))
			
			#正面範囲を加える
			#rangeList.append(CrosswideRange(const.LEFT_FRONT_AREA_X,const.RIGHT_FRONT_AREA_X))
			
			#各範囲において startX < endX となるようソート
			for eachRange in rangeList:
				eachRange.startX,eachRange.endX = sortNums(eachRange.startX,eachRange.endX)
			
			#視聴範囲をx座標が小さい順に並び替える
			rangeList = sorted(rangeList,cmp=lambda range1,range2: cmp(range1.startX,range2.startX))#cmp=self.cmpRangeByStartX

			
			#包括関係の範囲があれば小さい方を消去
			loopFinishFlag = False
			while loopFinishFlag is False:
				for index in range(len(rangeList)-1):
					if rangeList[index].endX > rangeList[index+1].endX:
						rangeList.pop(index+1)
						break
				loopFinishFlag = True
			
			#前の範囲の終点〜次の範囲の始点の間を暗くするエリアとする(幅が0以下なら何も加えない)
			darkFilterRangeList = []
			darkFilterRangeList.append(CrosswideRange(const.CAM_IMG_START_X,rangeList[0].startX))
			for index in range(len(rangeList)-1):
				if rangeList[index+1].startX > rangeList[index].endX:
					darkFilterRangeList.append(CrosswideRange(rangeList[index].endX,rangeList[index+1].startX))
			darkFilterRangeList.append(CrosswideRange(rangeList[len(rangeList)-1].endX,const.CAM_IMG_END_X))
	
			for eachRange in darkFilterRangeList:
				self.paintRect(event, self.getColor(const.FILTER_DRAW_COLOR_STR, const.FILTER_UNLISTEN_ALPHA), eachRange)


	#矩形を描画
	#引数：color 矩形の色(QColor)
	#      startX,endX 矩形の始点、終点
	def paintRect(self,event,color,listenRange):
		if listenRange is not None:
			painter = QtGui.QPainter()
			painter.begin(self)
			painter.setBrush(color)
			painter.drawRect(self.getPaintRect(listenRange.startX,listenRange.endX))

	def paintRecogWord(self,event):
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
		self.paintRecogWord(event)
		self.paintLocVoice()
		self.paintCamImg(event)
		self.paintListenRanges(event)
		self.paintDarkFilter(event)

		
		#tmpLocSrcList = global_var.locSrcList[:]
		#tmpVanLocSrcList= global_var.vanLocSrcList[:]		

	#マウスクリック時のイベント
	def mousePressEvent(self,event):
		self.ifDragging = True
		if manipulate_turtlebot2.isRotating() is not True:
			print "single click"
				
			if event.button() == Qt.LeftButton:
				#最大選択数分だけ選択されていなければ選択数を増やす
				if global_var.listenSeparateSoundCount < const.LISTEN_SEPARATE_SOUND_MAX_NUM:
					global_var.listenSeparateSoundCount += 1
		
				listenRange = global_var.listenRangeList[global_var.listenSeparateSoundCount -1]
				listenRange.startX = event.x()
				listenRange.endX = listenRange.startX
	
			elif event.button() == Qt.RightButton:
				#サブ範囲上でクリックされたらサブとメインを入れ替える
				if len(global_var.listenRangeList) == const.LISTEN_SEPARATE_SOUND_MAX_NUM:
					startX = global_var.listenRangeList[const.SUB_LISTEN_AREA].startX
					endX = global_var.listenRangeList[const.SUB_LISTEN_AREA].endX
					if ifNumBetween(startX,endX,event.x()):
						format_loc_src_microcone.switchListenRange()
				
				#メインの視聴範囲があればその方向を向く
				if len(global_var.listenRangeList) > 0:
					manipulate_turtlebot2.autoRotateStarter()
			
			#スクリーンショット
			#p = QtGui.QPixmap.grabWindow(self.winId())
			#p.save("scrshot"+str(time.time()),"png")

	#ダブルクリック
	def mouseDoubleClickEvent(self,event):
		if manipulate_turtlebot2.isRotating() is not True:
			print "double click"
			format_loc_src_microcone.listenWholeSound()
			#global_var.listenSeparateSoundFlag = False
			global_var.listenSeparateSoundCount = 0
			initListenRange()
			self.ifDoubleClicked = True

	#カーソル移動
	def mouseMoveEvent(self,event):
		if manipulate_turtlebot2.isRotating() is not True:
			global_var.listenRangeList[global_var.listenSeparateSoundCount - 1].endX = event.x()

	#クリック離し
	def mouseReleaseEvent(self, e):
		self.ifDragging = False
		
		if manipulate_turtlebot2.isRotating() is not True:
			#ダブルクリック後は実行しない
			if self.ifDoubleClicked is not True:
				listenRange = global_var.listenRangeList[global_var.listenSeparateSoundCount - 1]
				
				#座標を角度に変換してグローバル変数にセット
				if math.fabs(listenRange.endX - listenRange.startX) > const.IGNOR_PIX_THR:
					
					listenRange.startX,listenRange.endX = sortNums(listenRange.startX,listenRange.endX)
		
					listenRange.startAzimuth = thetaimg.getAzimuthFromXAxis(listenRange.startX)
					listenRange.endAzimuth = thetaimg.getAzimuthFromXAxis(listenRange.endX)
		
					format_loc_src_microcone.listenSeparateSound()
					
				else:
					format_loc_src_microcone.decListenSeparateSoundCount()
		
				print "listenSoundNum:"+str(global_var.listenSeparateSoundCount)
			else:
				self.ifDoubleClicked = False


	#視聴範囲用矩形枠取得
	def getPaintRect(self,startX,endX):
		absRange = math.fabs(endX - startX)
		
		if startX < endX:
			return QRect(startX,const.CAM_IMG_OFS_Y,absRange,const.CAM_IMG_HT)
		else:
			return QRect(endX,const.CAM_IMG_OFS_Y,absRange,const.CAM_IMG_HT)


#ウィンドウ外枠のクラス
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
		#print "isAutoRepeat:" + str(event.isAutoRepeat())
		if event.isAutoRepeat() is False:
			print "keyPressed"
			key = event.key()
			command = None
			if key == Qt.Key_Right:
				if global_var.rightKeyPressFlag is False:
					print "RIGHT"
					global_var.robotMoveDirection = const.RIGHT
					manipulate_turtlebot2.checkManualRotatingFlag()
					global_var.rightKeyPressFlag = True
			elif key == Qt.Key_Left:
				print "LEFT"
				manipulate_turtlebot2.moveRobot(const.JOY_LEFT)
			elif key == Qt.Key_Up:
				print "UP"
				manipulate_turtlebot2.moveRobot(const.JOY_UP)
			elif key == Qt.Key_Down:
				print "DOWN"
				manipulate_turtlebot2.moveRobot(const.JOY_DOWN)
			elif key == Qt.Key_Space:
				print "SPACE"
				manipulate_turtlebot2.rotateFinisher()
				format_loc_src_microcone.listenWholeSound()
				format_loc_src_microcone.initSeparateListenParam()

	
	def keyReleaseEvent(self,event):
		if event.isAutoRepeat() is False:
			print "keyReleased"
			key = event.key()
			command = None
			if key == Qt.Key_Right:
				print "RIGHT"
				#sendCommand(const.STOP_ROT_CMD)
				global_var.robotMoveDirection = const.STAY
				global_var.rightKeyPressFlag = False
			elif key == Qt.Key_Left:
				print "LEFT"
				manipulate_turtlebot2.moveRobot(const.JOY_LEFT)
			elif key == Qt.Key_Up:
				print "UP"
				manipulate_turtlebot2.moveRobot(const.JOY_UP)
			elif key == Qt.Key_Down:
				print "DOWN"
				manipulate_turtlebot2.moveRobot(const.JOY_DOWN)
		

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
	initListenRange()
	os.system(const.SET_TO_STR + str(const.MAN_ROT_TO))
	#signal.signal(signal.SIGINT, signal_handler)
	return app,window

#トピック購読処理
def subscriber():
	rospy.init_node(const.SYSTEM_NAME, anonymous = True)
	#kinect_tf.subscriber()
	format_loc_src_microcone.subscriber()
	recogword.subscriber()
	cameraimage.subscriber()
	manipulate_turtlebot2.initializer()
	#rospy.spin()

#メイン関数
if __name__ == '__main__':
	subscriber()
	#代入してウィンドウ情報を取得しないと上手く動かない
	app,window = initialize()
	#このメソッドで描画ループがスタート(一番最後に実行する)
	app.exec_()
	#sys.exit(app.exec_())
