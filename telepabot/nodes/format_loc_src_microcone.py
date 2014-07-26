#! /usr/bin/env python
# -*-coding: utf-8 -*-

#import roslib; roslib.load_manifest('ui_alt')
import rospy
import global_var
import copy
import const
import time
#import hark_msgs  # @UnresolvedImport
from hark_msgs.msg import HarkSourceVal  # @UnresolvedImport
from hark_msgs.msg import HarkSrcWave  # @UnusedImport @UnresolvedImport
from hark_msgs.msg import HarkSrcWaveVal  # @UnusedImport @UnresolvedImport
from hark_msgs.msg import HarkSource  # @UnresolvedImport
import math
import thetaimg
import csvlog
from PyQt4.QtCore import *
from PyQt4.QtGui import *


#定位した音源情報を格納するクラス
class LocSrc():
	def __init__(self,srcId,azimuth,power,drawBottomLeft,powerCode):
		self.srcId, self.azimuth, self.power, self.drawBottomLeft,self.powerCode = srcId,azimuth,power,drawBottomLeft,powerCode

#角度を-180~180に変換
def getPm180Azimuth(azimuth):
	if azimuth < -180:
		azimuth = azimuth + 360
	elif azimuth > 180:
		azimuth = azimuth - 360
	return azimuth

#microcone用(microconeの正面が0、左が＋、右がー)に角度を変換
def getMcAzimuth(azimuth):
	return getPm180Azimuth(-azimuth - const.MIC_ROTATE)

#UI用(左端が-180,右端が+180)に角度を変換
def getUIAzimuth(azimuth):
	return getPm180Azimuth(-(azimuth + const.MIC_ROTATE))


#format localization information
def localization_callback(data):
	global_var.harkSource = data
	global_var.locSrcList = []
	for index in range(len(data.src)):
		#print "mcAzimuth:"+str(data.src[index].azimuth)
		azimuth = getUIAzimuth(data.src[index].azimuth)
		#print "uiAzimuth:"+str(azimuth)
		
		x = thetaimg.getXAxisFromAzimuth(azimuth) - const.LOC_STR_WIDTH / 2 
		
		#print "azimuth:"+str(azimuth)
		#print "getXAxis"+str(thetaimg.getXAxisFromAzimuth(azimuth))
		#print "x:"+str(x)
		
		if x < 0:
			x = 0
		y = const.LOC_STR_Y_POS
		src = LocSrc(data.src[index].id,azimuth,data.src[index].power,QPoint(x,y),0)
		global_var.locSrcList.append(src)

def getAgainstAzimuthInPm180(azimuth):
	if azimuth > 0:
		return azimuth - 180
	else:
		return azimuth + 180

def getHarkSourceVal(sourceid,azimuth):
	sourceVal = HarkSourceVal()
	sourceVal.id = sourceid
	sourceVal.azimuth = azimuth
	sourceVal.x = math.cos(math.radians(azimuth))
	sourceVal.y = math.sin(math.radians(azimuth))
	return sourceVal

def getListenRangeSource(startAzimuth,endAzimuth):
	# ensure startAzimuth < endAzimuth
	if startAzimuth > endAzimuth:
		tmp = startAzimuth
		startAzimuth = endAzimuth
		endAzimuth = tmp
	#print "UI_startAzimuth:"+str(startAzimuth)
	#startAzimuth = getMcAzimuth(startAzimuth)
	#print "MC_startAzimuth:"+str(startAzimuth)
	#endAzimuth = getMcAzimuth(endAzimuth)
	
	selectorSource = HarkSource()
	separateSource = HarkSource()
	sourceid = 0
	azimuthRange = endAzimuth - startAzimuth

	# make sound source for input listen range to separation node	
	if azimuthRange < const.HARK_SEPARATION_RESOLUTION:
		azimuth = (startAzimuth + endAzimuth) / 2
		selectorSource.src.append(getHarkSourceVal(sourceid, getMcAzimuth(azimuth)))
	else:
		sourcePointCount = int((azimuthRange - const.HARK_SEPARATION_RESOLUTION) / const.HARK_SEPARATION_RESOLUTION + 3)
		azimuth = startAzimuth + const.HARK_SEPARATION_RESOLUTION / 2
		for sourceid in range(0,sourcePointCount):
			selectorSource.src.append(getHarkSourceVal(sourceid, getMcAzimuth(azimuth)))
			azimuth = azimuth + (azimuthRange - 30) / (sourcePointCount -1)
	
	separateSource = copy.deepcopy(selectorSource)
	print "addSrcId:"+str(sourceid)
	separateSource.src.append(getHarkSourceVal(sourceid + 1, getAgainstAzimuthInPm180(getMcAzimuth(azimuth))))
	selectorSource.exist_src_num = len(selectorSource.src)
	separateSource.exist_src_num = len(separateSource.src)

	#定位結果を使う
	selectorSource = getSoundSrcInRange()
	
	#print "selectorSource:"+str(selectorSource)
	
	return [selectorSource,separateSource]

def listenSeparateSound():
	selectorSource,separateSource = getListenRangeSource(global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)
	const.SEPARATE_SOURCE_PUB.publish(separateSource)
	const.SELECTOR_SOURCE_PUB.publish(selectorSource)
	const.SEP_LIS_TRIG_PUB.publish(True)
	#r = rospy.Rate(3)
	#r.sleep()
	csvlog.writeLog(const.CSV_START_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

def getSoundSrcInRange():
	harkSource = copy.deepcopy(global_var.harkSource)
	msg_select = HarkSource()
	for index in range(len(harkSource.src)):
		if ifThetaInRange(harkSource.src[index].azimuth):
			#append = HarkSourceVal()
			#append.id = harkSource.src[index].id
			#append.azimuth = harkSource.src[index].azimuth
			msg_select.src.append(harkSource.src[index])
			msg_select.exist_src_num += 1
	global_var.msg_select_gl = msg_select
	return msg_select

def listenWholeSound():
	const.SEP_LIS_TRIG_PUB.publish(False)
	csvlog.writeLog(const.CSV_END_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

#reverse angle for microcone(microcone's right angle is minus
def getListenAngle(xaxis):
	return -((xaxis * 2 * const.IMG_HOR_HALF_VIEW_AGL)/const.CAM_WHOLE_IMG_WID - const.IMG_HOR_HALF_VIEW_AGL)

def setListenAngles(startX,endX):
	global_var.listenRangeStartAngle = thetaimg.getAzimuthFromXAxis(startX)
	global_var.listenRangeEndAngle = thetaimg.getAzimuthFromXAxis(endX)

def makeSoundSrcInRange():
	msg_select = HarkSource()
	return msg_select


#角度が音源視聴範囲内か判定する
def ifThetaInRange(theta):
	if getUIAzimuth(theta) >= global_var.listenRangeStartAngle and getUIAzimuth(theta) <= global_var.listenRangeEndAngle:
		return True
	else :
		return False


#トピック購読処理
def subscriber():
	rospy.Subscriber(const.HARK_LOC_SOURCE_TOPIC_NAME, HarkSource, localization_callback, buff_size = 1)
