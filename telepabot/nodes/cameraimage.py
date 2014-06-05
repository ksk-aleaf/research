#! /usr/bin/env python
# -*-coding: utf-8 -*-



import roslib; roslib.load_manifest("telepabot")
import rospy
import copy
import const
import types


#pyqt import
from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

#openCV import
from cv_bridge import CvBridge, CvBridgeError
import cv


import numpy

#ros camera image msg
from sensor_msgs.msg import Image

#グローバル変数モジュール
import global_var
#定数モジュール
import const

#camera image class
class CameraDevice(QtCore.QObject):

	iplimageSygnal = QtCore.pyqtSignal(cv.iplimage)

	def __init__(self, mirrored=False, parent=None):

		super(CameraDevice, self).__init__(parent)

		self.mirrored = mirrored
		
		#signal timer for slot(_queryFrame)
		self.signalTimer = QtCore.QTimer(self)
		self.signalTimer.timeout.connect(self.emitImage)
		self.signalTimer.setInterval(const.CAM_DRAW_HZ)
		
		self.paused = False

	@property
	def paused(self):
		return not self.signalTimer.isActive()

	@paused.setter
	def paused(self, p):
		if p:
			self.signalTimer.stop()
		else:
			self.signalTimer.start()

	
	@QtCore.pyqtSlot()
	def emitImage(self):
		frame = global_var.cvCenterImage

		if frame == None:
			print "frame = None"
			return

		if self.mirrored:
			mirroredFrame = cv.CreateImage(cv.GetSize(frame), 8, 3)
			cv.Copy(frame, mirroredFrame, None)
			#cv.Flip(frame, mirroredFrame, 1)
			frame =  mirroredFrame
		
		if frame == None:
			print "frame = None 2"
			return
		
		global_var.cvCenterImage = frame
		#self.iplimageSygnal.emit(frame)



#OpenCVIplImageからPyQtのQImageへの変換クラス http://rafaelbarreto.wordpress.com/tag/pyqt/
class OpenCVQImage(QtGui.QImage):
	def __init__(self, opencvRgbImg):
		w,h = cv.GetSize(opencvRgbImg)
		super(OpenCVQImage, self).__init__(opencvRgbImg.tostring(),w , h, QtGui.QImage.Format_RGB888)

def getGrayScale(cvmatImage):
	for row in range(cvmatImage.rows):
		for col in range(cvmatImage.cols):
			if cvmatImage[row,col][0] + 50 < 255:
				cvmatImage[row,col]=(cvmatImage[row,col][0] + 50,cvmatImage[row,col][1],cvmatImage[row,col][2])
	return cvmatImage



def drawCameraImage(event,cvImage,point,painter):
	#cvImage = getGrayScale(cvImage)
	if cvImage is not None:
		painter.drawImage(point, OpenCVQImage(cvImage))


#receive image callback
def imageCallback(rosImage):
	global_var.cvCenterImage = const.CV_BRIDGE.imgmsg_to_cv(rosImage, "rgb8")
	if global_var.cvCenterImage is None:
		print "none"



def subscriber():
	rospy.Subscriber(const.CENTER_CAM_IMG_TOPIC_NAME, Image, imageCallback)
	#rospy.Subscriber(const.CAM_IMG_TOPIC_NAME, Image, imageCallback)
