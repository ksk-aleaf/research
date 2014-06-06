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


#OpenCVIplImageからPyQtのQImageへの変換クラス http://rafaelbarreto.wordpress.com/tag/pyqt/
class OpenCVQImage(QtGui.QImage):
	def __init__(self, opencvRgbImg, format):
		w,h = cv.GetSize(opencvRgbImg)
		super(OpenCVQImage, self).__init__(opencvRgbImg.tostring(),w , h, format)

def getGrayScale(cvmatImage):
	for row in range(cvmatImage.rows):
		for col in range(cvmatImage.cols):
			if cvmatImage[row,col][0] + 50 < 255:
				cvmatImage[row,col]=(cvmatImage[row,col][0] + 50,cvmatImage[row,col][1],cvmatImage[row,col][2])
	return cvmatImage



def drawCameraImage(event,cvImage,format,point,painter):
	if cvImage is not None:
		painter.drawImage(point, OpenCVQImage(cvImage,format))


#receive center image callback
def centerImageCallback(rosImage):
	global_var.cvCenterImage = const.CV_BRIDGE.imgmsg_to_cv(rosImage, "rgb8")
	
#receive left image callback
def leftImageCallback(rosImage):
	global_var.cvLeftImage = const.CV_BRIDGE.imgmsg_to_cv(rosImage, "rgb8")

#receive right image callback
def rightImageCallback(rosImage):
	global_var.cvRightImage = const.CV_BRIDGE.imgmsg_to_cv(rosImage, "rgb8")


def subscriber():
	rospy.Subscriber(const.CENTER_CAM_IMG_TOPIC_NAME, Image, centerImageCallback)
	rospy.Subscriber(const.LEFT_CAM_IMG_TOPIC_NAME, Image, leftImageCallback)
	rospy.Subscriber(const.RIGHT_CAM_IMG_TOPIC_NAME, Image, rightImageCallback)
