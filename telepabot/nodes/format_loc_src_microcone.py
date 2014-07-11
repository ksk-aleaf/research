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
#from std_msgs.msg import Bool
import csvlog

#定位した音源情報を格納するクラス
class LocSrc():
	def __init__(self,srcId,azimuth,power,draw_left_x,draw_top_y,powerCode):
		self.srcId, self.azimuth, self.power, self.draw_left_x, self.draw_top_y,self.powerCode = srcId,azimuth,power,draw_left_x,draw_top_y,powerCode
# class LocSrc():
# 	def __init__(self,srcId,theta,power,x_onImage,y_arrow,onImageFlag,powerCode):
# 		self.srcId, self.theta, self.power, self.x_onImage, self.y_arrow, self.onImageFlag,self.deletedTime,self.powerCode = srcId,theta,power,x_onImage,y_arrow,onImageFlag,0,powerCode



#音源の角度を引数にとり、付与するIDを返す
# def getSourceDirectionId(theta):
# 	if(theta >= global_var.listenRangeStartAngle and theta < global_var.listenRangeEndAngle):
# 		return const.LISTENABLE
# 	else:
# 		return const.UNLISTENABLE

#Show localization information
def localization_callback(data):
	for index in range(len(data.src)):
		azimuth = data.src[index].azimuth
		x = thetaimg.getXAxisFromAzimuth(azimuth) - const.LOC_STR_WIDTH / 2
		if x < 0:
			x = 0
		y = const.LOC_STR_Y_POS
		src = LocSrc(data.src[index].id,azimuth,data.src[index].power,x,y,0)
		global_var.locSrcList.append(src)
	#global_var.locSrcList = data
	#global_var.locSrcList = []

# 	#音源リスト内の最大パワーを求める
# 	if len(data.src) > 0:
# 		global_var.max_power = data.src[0].power
# 		for j in range(len(data.src)):
# 			if data.src[j].power - global_var.max_power > 0:
# 				global_var.max_power = data.src[j].power
# 
# 	#harkSourceに画像上のx座標情報を追加してglobal_var.locSrcListに格納
# 	for i in range(len(data.src)):
# 		if(data.src[i].azimuth >= -const.IMG_HOR_HALF_VIEW_AGL and data.src[i].azimuth <= const.IMG_HOR_HALF_VIEW_AGL):#映像視野内
# 			x_onImage = (const.CAM_WHOLE_IMG_WID / 2)*(1-(math.tan(math.radians(data.src[i].azimuth))/math.tan(math.radians(const.IMG_HOR_HALF_VIEW_AGL))))
# 			onImageFlag = True
# 			y_arrow = -100
# 		else:#画面外
# 			if(data.src[i].azimuth > 0):
# 				y_arrow = 0#const.PIXEL_BY_STRIDE*(int( (data.src[i].azimuth - const.IMG_HOR_HALF_VIEW_AGL) / const.THETA_STRIDE ))
# 			else:
# 				y_arrow = 0#-const.PIXEL_BY_STRIDE*(int( (data.src[i].azimuth + const.IMG_HOR_HALF_VIEW_AGL) / const.THETA_STRIDE ))
# 			x_onImage = -100
# 			onImageFlag = False
# 
# 		if (global_var.max_power - data.src[i].power) < const.STRONG_POW_THR:
# 			powerCode = const.STRONG_POW_CODE
# 		elif (global_var.max_power - data.src[i].power) < const.MEDIUM_POW_THR:
# 			powerCode = const.MEDIUM_POW_CODE
# 		else:
# 			powerCode = const.WEAK_POW_CODE
# 
# 		src = LocSrc(data.src[i].id, data.src[i].azimuth, data.src[i].power,x_onImage,y_arrow,onImageFlag,powerCode)
# 		global_var.locSrcList.append(src)
# 
# 
# 
# 	#消えた音源があればリストに格納
# 	for prevIndex in range(len(global_var.prevLocSrcList)):
# 		#if not prevLocSrcList[prevIndex].onImageFlag:
# 		flag = False
# 		for locIndex in range(len(global_var.locSrcList)):
# 			if(global_var.prevLocSrcList[prevIndex].srcId == global_var.locSrcList[locIndex].srcId):
# 				flag = True
# 		if not flag:
# 			global_var.prevLocSrcList[prevIndex].deletedTime = time.time()
# 			global_var.vanLocSrcList.append(global_var.prevLocSrcList[prevIndex])
# 
# 	global_var.prevLocSrcList = global_var.locSrcList[:]#リストの中身をコピー
# 
# 	#消えて五秒経過した音源は削除
# 	tmpList = []
# 	for index in range(len(global_var.vanLocSrcList)):
# 		if (time.time() - global_var.vanLocSrcList[index].deletedTime) < const.VAN_SRC_KEEP_SEC:
# 			tmpList.append(global_var.vanLocSrcList[index])
# 	global_var.vanLocSrcList= tmpList[:]
# 
# 	#存在する音源と角度が被っていたら消えた音源の情報は削除
# 	tmpList = []
# 	for vanishIndex in range(len(global_var.vanLocSrcList)):
# 		deleteFlag = False
# 		for locIndex in range(len(global_var.locSrcList)):
# 			if global_var.locSrcList[locIndex].y_arrow == global_var.vanLocSrcList[vanishIndex].y_arrow:
# 				deleteFlag = True
# 		if not deleteFlag:
# 			tmpList.append(global_var.vanLocSrcList[vanishIndex])
# 	global_var.vanLocSrcList= tmpList[:]


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
	
	selectorSource = HarkSource()
	separateSource = HarkSource()
	sourceid = 0
	azimuthRange = endAzimuth - startAzimuth

	# make sound source for input listen range to separation node	
	if azimuthRange < const.HARK_SEPARATION_RESOLUTION:
		azimuth = (startAzimuth + endAzimuth) / 2
		selectorSource.src.append(getHarkSourceVal(sourceid, azimuth))
	else:
		sourcePointCount = int((azimuthRange - const.HARK_SEPARATION_RESOLUTION) / const.HARK_SEPARATION_RESOLUTION + 3)
		azimuth = startAzimuth + const.HARK_SEPARATION_RESOLUTION / 2
		for sourceid in range(0,sourcePointCount):
			selectorSource.src.append(getHarkSourceVal(sourceid, azimuth))
			azimuth = azimuth + (azimuthRange - 30) / (sourcePointCount -1)
	
	separateSource = copy.deepcopy(selectorSource)
	print "addSrcId:"+str(sourceid)
	separateSource.src.append(getHarkSourceVal(sourceid + 1, getAgainstAzimuthInPm180((startAzimuth + endAzimuth) / 2)))
	selectorSource.exist_src_num = len(selectorSource.src)
	separateSource.exist_src_num = len(separateSource.src)
	
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

#音源視聴範囲内の音源情報を返す
# def getSoundSrcInRange(harkSource):
# 	msg_select = HarkSource()
# 	for index in range(len(harkSource)):
# 		if ifThetaInRange(harkSource[index].azimuth):
# 			append = HarkSourceVal()
# 			append.id = getSourceDirectionId(harkSource[index].azimuth)
# 			append.azimuth = harkSource[index].azimuth
# 			msg_select.src.append(append)
# 			msg_select.exist_src_num += 1
# 	return msg_select

#角度が音源視聴範囲内か判定する
def ifThetaInRange(theta):
	if theta >= global_var.listenRangeStartAngle and theta <= global_var.listenRangeEndAngle:
		return True
	else :
		return False


#トピック購読処理
def subscriber():
	rospy.Subscriber(const.HARK_LOC_SOURCE_TOPIC_NAME, HarkSource, localization_callback, buff_size = 1)
