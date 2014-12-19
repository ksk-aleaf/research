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
import telepabot

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


#定位情報コールバック関数
#定位情報を描画用にフォーマット
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

	if isSeparateListening() is True:
		sendSrcForSoundSeparation()

#±180度の範囲で反対側の角度を取得
def getAgainstAzimuthInPm180(azimuth):
	if azimuth > 0:
		return azimuth - 180
	else:
		return azimuth + 180

#角度からHarkSourceValインスタンスを生成
#x,yをちゃんと入れないとharkの定位・分離周りが上手く動かない
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
	
	separateSource = global_var.harkSource
	selectorSource = getSoundSrcInListenRange(separateSource)

	return [selectorSource,separateSource]

#harkの音源分離モジュールに音源情報を送る
def sendSrcForSoundSeparation():
	separateSource = global_var.harkSource
	selectorSource = getSoundSrcInListenRange(separateSource)
	const.SEPARATE_SOURCE_PUB.publish(separateSource)
	const.SELECTOR_SOURCE_PUB.publish(selectorSource)

#分離音声視聴モードに切り替える
# def listenSeparateSound():
# 	#global_var.listenSeparateSoundFlag = True
# 	const.SEP_LIS_TRIG_PUB.publish(True)
	#csvlog.writeLog(const.CSV_START_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

#HARKの分離音声視聴フラグの切り替え
def checkListenSoundMode():
	if isSeparateListening():
		const.SEP_LIS_TRIG_PUB.publish(True)
	else:
		const.SEP_LIS_TRIG_PUB.publish(False)

def isSeparateListening():
	if global_var.mainListenRange is not None and global_var.subListenRange is not None:
		if global_var.mainListenRange.selectFlag is True or global_var.subListenRange.selectFlag is True:
			return True
	return False

#視聴範囲内のharkSourceを返す
#引数：全定位情報
#返り値：視聴範囲内の定位情報
def getSoundSrcInListenRange(harkSource):
	sourceInRange = HarkSource()
	for index in range(len(harkSource.src)):
		if ifThetaInRanges(harkSource.src[index].azimuth):
			sourceInRange.src.append(harkSource.src[index])
			sourceInRange.exist_src_num += 1
	return sourceInRange

#全体音声視聴モードに切り替える
# def listenWholeSound():
# 	#initSeparateListenParam()
# 	const.SEP_LIS_TRIG_PUB.publish(False)
# 	#csvlog.writeLog(const.CSV_END_LISTEN_TAG, global_var.listenRangeStartAngle, global_var.listenRangeEndAngle)

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
def ifThetaInRanges(theta):
	def ifThetaInRange(listenRange):
		#print "[ifThetaInRange]"
		#listenRange.printRangeInfo()
		if getUIAzimuth(theta) >= listenRange.startAzimuth - const.AZIMUTH_RANGE_BUF and getUIAzimuth(theta) <= listenRange.endAzimuth + const.AZIMUTH_RANGE_BUF:
			return True
	
#	flag = False

	if ifThetaInRange(global_var.mainListenRange) or ifThetaInRange(global_var.subListenRange):
		return True
	else:
		return False
# 	for index in range(global_var.listenSeparateSoundCount):
# 		listenRange = global_var.listenRangeList[index]
# 		if getUIAzimuth(theta) >= listenRange.startAzimuth - const.AZIMUTH_RANGE_BUF and getUIAzimuth(theta) <= listenRange.endAzimuth + const.AZIMUTH_RANGE_BUF:
# 			flag = True

#	return flag


#分離音声視聴範囲数を減らす
def decListenSeparateSoundCount():
	if global_var.listenSeparateSoundCount > 0:
		global_var.listenSeparateSoundCount -= 1
		if global_var.listenSeparateSoundCount == 0:
			listenWholeSound()
	else:
		print "decrement listen separate sound count error"

#視聴範囲の角度を引数の角度だけずらす
def shiftListenRange(azimuth):
	for listenRange in global_var.listenRangeList:
		listenRange.startAzimuth += azimuth
		listenRange.endAzimuth += azimuth
		listenRange.startX = thetaimg.getXAxisFromAzimuth(listenRange.startAzimuth)
		listenRange.endX = thetaimg.getXAxisFromAzimuth(listenRange.endAzimuth)

#メインとサブの視聴範囲を入れ替える
def switchListenRange():
	if len(global_var.listenRangeList) == const.LISTEN_SEPARATE_SOUND_MAX_NUM:
		global_var.listenRangeList[const.MAIN_LISTEN_AREA],	global_var.listenRangeList[const.SUB_LISTEN_AREA] = global_var.listenRangeList[const.SUB_LISTEN_AREA],	global_var.listenRangeList[const.MAIN_LISTEN_AREA]


#分離視聴角度(1番目)を取得
def getSeparateListenAngle():
	azimuth = 0
	if global_var.listenSeparateSoundCount > 0:
		azimuth = (global_var.listenRangeList[0].startAzimuth + global_var.listenRangeList[0].endAzimuth) /2
	return azimuth

#分離音声聴取範囲に角度をセット
# def setListenRangeAzimuth(listenRange):
# 	listenRange.startX,listenRange.endX = telepabot.sortNums(listenRange.startX,listenRange.endX)
# 	listenRange.startAzimuth = thetaimg.getAzimuthFromXAxis(listenRange.startX)
# 	listenRange.endAzimuth = thetaimg.getAzimuthFromXAxis(listenRange.endX)
	#listenRange.printRangeInfo()

#分離音声視聴に関するパラメータを初期化
def initSeparateListenParam():
	global_var.listenSeparateSoundCount = 0
	#global_var.listenSeparateSoundFlag = False

#debug用
def printListenRanges():
	def printListenRange(listenRange):
		print "startX:" + str(listenRange.startX)
		print "endX:" + str(listenRange.endX)
		print "startAzimuth:" + str(listenRange.startAzimuth)
		print "endAzimuth:" + str(listenRange.endAzimuth)
	print "main"
	printListenRange(global_var.mainListenRange)
	print "sub"
	printListenRange(global_var.subListenRange)

#トピック購読処理
def subscriber():
	rospy.Subscriber(const.HARK_LOC_SOURCE_TOPIC_NAME, HarkSource, localization_callback, buff_size = 1)
