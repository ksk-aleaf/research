#! /usr/bin/env python
# -*-coding: utf-8 -*-

#import roslib; roslib.load_manifest('ui_alt')
import rospy
import global_var
import copy
import const
import time
from hark_msgs.msg import HarkSourceVal
from hark_msgs.msg import HarkSrcWave
from hark_msgs.msg import HarkSrcWaveVal
from hark_msgs.msg import HarkSource
import math
import thetaimg
#import button
from std_msgs.msg import Bool

#定位した音源情報を格納するクラス
class LocSrc():
	def __init__(self,srcId,theta,power,x_onImage,y_arrow,onImageFlag,powerCode):
		self.srcId, self.theta, self.power, self.x_onImage, self.y_arrow, self.onImageFlag,self.deletedTime,self.powerCode = srcId,theta,power,x_onImage,y_arrow,onImageFlag,0,powerCode


#Show localization information
def localization_callback(data):
	global_var.locSrcList = []

	#音源リスト内の最大パワーを求める
	if len(data.src) > 0:
		global_var.max_power = data.src[0].power
		for j in range(len(data.src)):
			if data.src[j].power - global_var.max_power > 0:
				global_var.max_power = data.src[j].power

	#harkSourceに画像上のx座標情報を追加してglobal_var.locSrcListに格納
	for i in range(len(data.src)):
		if(data.src[i].theta >= -const.IMG_HOR_HALF_VIEW_AGL and data.src[i].theta <= const.IMG_HOR_HALF_VIEW_AGL):#映像視野内
			x_onImage = (const.CAM_IMG_WID / 2)*(1-(math.tan(math.radians(data.src[i].theta))/math.tan(math.radians(const.IMG_HOR_HALF_VIEW_AGL))))
			onImageFlag = True
			y_arrow = -100
		else:#画面外
			if(data.src[i].theta > 0):
				y_arrow = const.PIXEL_BY_STRIDE*(int( (data.src[i].theta - const.IMG_HOR_HALF_VIEW_AGL) / const.THETA_STRIDE ))
			else:
				y_arrow = -const.PIXEL_BY_STRIDE*(int( (data.src[i].theta + const.IMG_HOR_HALF_VIEW_AGL) / const.THETA_STRIDE ))
			x_onImage = -100
			onImageFlag = False

		if (global_var.max_power - data.src[i].power) < const.STRONG_POW_THR:
			powerCode = const.STRONG_POW_CODE
		elif (global_var.max_power - data.src[i].power) < const.MEDIUM_POW_THR:
			powerCode = const.MEDIUM_POW_CODE
		else:
			powerCode = const.WEAK_POW_CODE

		src = LocSrc(data.src[i].id, data.src[i].theta, data.src[i].power,x_onImage,y_arrow,onImageFlag,powerCode)
		global_var.locSrcList.append(src)



	#消えた音源があればリストに格納
	for prevIndex in range(len(global_var.prevLocSrcList)):
		#if not prevLocSrcList[prevIndex].onImageFlag:
		flag = False
		for locIndex in range(len(global_var.locSrcList)):
			if(global_var.prevLocSrcList[prevIndex].srcId == global_var.locSrcList[locIndex].srcId):
				flag = True
		if not flag:
			global_var.prevLocSrcList[prevIndex].deletedTime = time.time()
			global_var.vanLocSrcList.append(global_var.prevLocSrcList[prevIndex])

	global_var.prevLocSrcList = global_var.locSrcList[:]#リストの中身をコピー

	#消えて五秒経過した音源は削除
	tmpList = []
	for index in range(len(global_var.vanLocSrcList)):
		if (time.time() - global_var.vanLocSrcList[index].deletedTime) < const.VAN_SRC_KEEP_SEC:
			tmpList.append(global_var.vanLocSrcList[index])
	global_var.vanLocSrcList= tmpList[:]

	#存在する音源と角度が被っていたら消えた音源の情報は削除
	tmpList = []
	for vanishIndex in range(len(global_var.vanLocSrcList)):
		deleteFlag = False
		for locIndex in range(len(global_var.locSrcList)):
			if global_var.locSrcList[locIndex].y_arrow == global_var.vanLocSrcList[vanishIndex].y_arrow:
				deleteFlag = True
		if not deleteFlag:
			tmpList.append(global_var.vanLocSrcList[vanishIndex])
	global_var.vanLocSrcList= tmpList[:]

	msg = HarkSource()

	#音源情報のIDを振りなおす
	for k in range(len(data.src)):
		msg_val = HarkSourceVal()
		msg_val.id = button.getSourceDirectionId(data.src[k].theta)
		msg_val.x  = data.src[k].x
		msg_val.y  = data.src[k].y
		msg_val.theta = data.src[k].theta
		msg.src.append(msg_val)
		msg.exist_src_num += 1

	#処理後の音源情報を発行
	const.CLASSIFIED_SOURCE_PUB.publish(msg)
	global_var.prev_msg = msg
	
	#分離音声を視聴している場合は範囲内の音源情報を発行する
	const.SELECTED_SOURCE_PUB.publish(getSoundSrcInRange(data.src))


def listenSeparateSound():
	msg_select = HarkSource()
	msg_select.exist_src_num = 1

	append_msg = HarkSourceVal()
	append_msg.id = const.LISTENABLE
	append_msg.azimuth = (global_var.listenRangeStartAngle + global_var.listenRangeEndAngle)/2
	msg_select.src.append(append_msg)

	r = rospy.Rate(3)
	r.sleep()
	const.SEP_LIS_TRIG_PUB.publish(True)

def listenWholeSound():
	const.SEP_LIS_TRIG_PUB.publish(False)

def getListenAngle(xaxis):
	return (xaxis * 2 * const.IMG_HOR_HALF_VIEW_AGL)/const.CAM_IMG_WID - const.IMG_HOR_HALF_VIEW_AGL

def setListenAngles(startX,endX):
	global_var.listenRangeStartAngle = thetaimg.getThetaFromXAxis(startX)
	#thetaimg.getXAxisFromAzimuth(startX)
	# = getListenAngle(startX)
	global_var.listenRangeEndAngle = thetaimg.getThetaFromXAxis(endX)
	#thetaimg.getXAxisFromAzimuth(endX)
	# = getListenAngle(endX)

def makeSoundSrcInRange():
	msg_select = HarkSource()
	return msg_select

#音源視聴範囲内の音源情報を返す
def getSoundSrcInRange(harkSource):
	msg_select = HarkSource()
	for index in range(len(harkSource)):
		if ifThetaInRange(harkSource[index].theta):
			append = HarkSourceVal()
			append.id = button.getSourceDirectionId(harkSource[index].theta)
			append.theta = harkSource[index].theta
			msg_select.src.append(append)
			msg_select.exist_src_num += 1
	return msg_select

#角度が音源視聴範囲内か判定する
def ifThetaInRange(theta):
	if theta >= global_var.listenRangeStartAngle and theta <= global_var.listenRangeEndAngle:
		return True
	else :
		return False


#トピック購読処理
def subscriber():
	rospy.Subscriber(const.HARK_LOC_SOURCE_TOPIC_NAME, HarkSource, localization_callback, buff_size = 1)
