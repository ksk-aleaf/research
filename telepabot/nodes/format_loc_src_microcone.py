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
		azimuth = getUIAzimuth(data.src[index].azimuth)
		x = thetaimg.getXAxisFromAzimuth(azimuth) - const.LOC_STR_WIDTH / 2
		
		if x < 0:
			x = 0
		y = const.LOC_STR_Y_POS
		src = LocSrc(data.src[index].id,azimuth,data.src[index].power,QPoint(x,y),0)
		global_var.locSrcList.append(src)

	if global_var.listenSeparateSoundFlag is True:
		sendSrcForSoundSeparation()

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

#視聴範囲内に音源があるかどうかを取得
def ifSoundSrcInListenRange(harkSource):
	if len(harkSource) > 0:
		return True
	else:
		return False

#音源分離モジュールに送信する音源位置情報を返却
#引数
#startAzimuth:視聴範囲角度の始点
#endAzimuth:視聴範囲角度の終点
#返り値
#separateSource:GHDSSノードに入力するソース
#selectorSource:mapSelectorに入力するソース
def getSourceForSeparation(startAzimuth,endAzimuth):
	# ensure startAzimuth < endAzimuth
	if startAzimuth > endAzimuth:
		tmp = startAzimuth
		startAzimuth = endAzimuth
	
	separateSource = global_var.harkSource
	selectorSource = getSoundSrcInListenRange(separateSource)

	if len(selectorSource.src) == 0:
		# 視聴範囲内に音源が定位できない場合は、分離性能は落ちるが定位情報を生成する
		sourceid = 0
		azimuthRange = endAzimuth - startAzimuth
		if azimuthRange < const.HARK_SEPARATION_RESOLUTION:
			azimuth = (startAzimuth + endAzimuth) / 2
			selectorSource.src.append(getHarkSourceVal(sourceid, getMcAzimuth(azimuth)))
		else:
			sourcePointCount = int((azimuthRange - const.HARK_SEPARATION_RESOLUTION) / const.HARK_SEPARATION_RESOLUTION + 3)
			azimuth = startAzimuth + const.HARK_SEPARATION_RESOLUTION / 2
			for sourceid in range(0,sourcePointCount):
				selectorSource.src.append(getHarkSourceVal(sourceid, getMcAzimuth(azimuth)))
				azimuth = azimuth + (azimuthRange - const.HARK_SEPARATION_RESOLUTION) / (sourcePointCount -1)
			separateSource = copy.deepcopy(selectorSource)
			separateSource.src.append(getHarkSourceVal(sourceid + 1, getAgainstAzimuthInPm180(getMcAzimuth(azimuth))))
			selectorSource.exist_src_num = len(selectorSource.src)
			separateSource.exist_src_num = len(separateSource.src)

	return [selectorSource,separateSource]

#harkの音源分離モジュールに音源情報を送る
def sendSrcForSoundSeparation():
	selectorSource,separateSource = getSourceForSeparation(global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)
	const.SEPARATE_SOURCE_PUB.publish(separateSource)
	const.SELECTOR_SOURCE_PUB.publish(selectorSource)	

def listenSeparateSound():
	global_var.listenSeparateSoundFlag = True
	const.SEP_LIS_TRIG_PUB.publish(True)
	#csvlog.writeLog(const.CSV_START_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

def getSoundSrcInListenRange(harkSource):
	sourceInRange = HarkSource()
	for index in range(len(harkSource.src)):
		if ifThetaInRange(harkSource.src[index].azimuth):
			sourceInRange.src.append(harkSource.src[index])
			sourceInRange.exist_src_num += 1
	return sourceInRange

def listenWholeSound():
	global_var.listenSeparateSoundFlag = False
	const.SEP_LIS_TRIG_PUB.publish(False)
	csvlog.writeLog(const.CSV_END_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

#マイクロコーンは右側がマイナスなので角度を左右反転
def getListenAngle(xaxis):
	return -((xaxis * 2 * const.IMG_HOR_HALF_VIEW_AGL)/const.CAM_WHOLE_IMG_WID - const.IMG_HOR_HALF_VIEW_AGL)


def setListenAngles(startX,endX):
	global_var.listenRangeStartAngle = thetaimg.getAzimuthFromXAxis(startX)
	global_var.listenRangeEndAngle = thetaimg.getAzimuthFromXAxis(endX)

def setListenAxis(startAzimuth,endAzimuth):
	global_var.listenRangeStartX = thetaimg.getXAxisFromAzimuth(startAzimuth)
	global_var.listenRangeEndX = thetaimg.getXAxisFromAzimuth(endAzimuth)

#角度が音源視聴範囲内か判定する
def ifThetaInRange(theta):
	if getUIAzimuth(theta) >= global_var.listenRangeStartAngle and getUIAzimuth(theta) <= global_var.listenRangeEndAngle:
		return True
	else :
		return False


#トピック購読処理
def subscriber():
	rospy.Subscriber(const.HARK_LOC_SOURCE_TOPIC_NAME, HarkSource, localization_callback, buff_size = 1)
