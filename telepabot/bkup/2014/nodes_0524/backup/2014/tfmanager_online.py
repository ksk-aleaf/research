#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('ui_alt')
import rospy
import tf
import math
import sys
from ui_alt.msg import tf_uialt, src_tf_uialt
import const
 
 #Frame Name
OPENNI_DEPTH_FRAME = "/openni_depth_frame"
USER_NUM = 4
FIRST_USER_NUM = 1

#Frame Trans Index
X = 0
Y = 1
Z = 2
 
 
BODY_FRAMES = [
	'head',
	'neck',
	'torso'
]

#Duration
ROSPY_DURATION = rospy.Duration()
#SLEEP_RATE = rospy.Rate()

#Detect User Lost
DEFAULT_COUNT = 10
USER_LOST = 0

#Publisher
PUB_USER1 = rospy.Publisher('tf_processed1', tf_uialt)
PUB_USER2 = rospy.Publisher('tf_processed2', tf_uialt)
PUB_USER3 = rospy.Publisher('tf_processed3', tf_uialt)
PUB_USER4 = rospy.Publisher('tf_processed4', tf_uialt)
PUB_USERS = [PUB_USER1,PUB_USER2,PUB_USER3,PUB_USER4]

#Message
EMPTY_MSG = tf_uialt()

def getTopicName(userIdx,frame):
	return "/" + frame + str(userIdx+1)

#def getUserRange():
#	return [userIdx+1 for userIdx in range(USER_NUM)]

def ifSameTrans(trans1,trans2):
	print "trans1"
	print trans1
	print "trans2"
	print trans2
	if not trans1 or not trans2:
		return False
	elif trans1[X] == trans2[X]:
		return True
#	elif (trans1.[X] == trans2.[X] and trans1.[Y] == trans2.[Y]) and trans1.[Z] == trans2.[Z]:
#		return true
	else :
		return False

#kinect映像上のX座標,Y座標,角度情報を持つメッセージを返す
#trans：トラッキングデータ(X,Y,Z)
#topicName:トピック名
def getPubMsg(trans,topicName):
	append = src_tf_uialt()
	append.parts = topicName
	append.x	 = const.CAM_IMG_WID - const.CAM_IMG_HALF_WID * (1 - (math.tan(trans[X] / trans[Z]) / math.tan(math.radians(const.IMG_HOR_HALF_VIEW_AGL))))
	append.y	 = const.CAM_IMG_HALF_HT * (1 - (math.tan(trans[Y] / trans[Z]) / math.tan(math.radians(const.IMG_ELEV_HALF_VIEW_AGL))))
	append.theta = -math.degrees(math.atan(trans[X] / trans[Z]))
	return append


if __name__ == ('__main__'):

	rospy.init_node('tf_manager')
	pub_msgs = []
	listener = tf.TransformListener()
	prev_trans = []
	userUndetectedCount = DEFAULT_COUNT
	SLEEP_RATE = rospy.Rate(5)

	while not rospy.is_shutdown():
		#発行メッセージの初期化
		pub_msgs = []
		for userIdx in range(USER_NUM):
			pub_msgs.append(tf_uialt())

		if listener.frameExists(OPENNI_DEPTH_FRAME):
			print "frameExist"
			for userIdx in range(USER_NUM):
				for frame in BODY_FRAMES:
					topicName = getTopicName(userIdx,frame)
					if listener.frameExists(topicName):
						#部位の3次元座標を取得
						trans, rot = listener.lookupTransform(OPENNI_DEPTH_FRAME, topicName, ROSPY_DURATION)
						#kinect映像上の座標、角度を格納したメッセージを発行リストに追加
						pub_msgs[userIdx].src.append(getPubMsg(trans,topicName))
						#座標値が変わっていなければuserLostCountを減らす
						if ifSameTrans(trans,prev_trans):
							if userUndetectedCount > USER_LOST:
								userUndetectedCount -= 1
						else:
							prev_trans = trans
							userLostCount = DEFAULT_COUNT
			
			if userUndetectedCount is USER_LOST:
				for userIdx in range(USER_NUM):
					PUB_USERS[userIdx].publish(EMPTY_MSG)
			else:
				for userIdx in range(USER_NUM):
					PUB_USERS[userIdx].publish(pub_msgs[userIdx])

			SLEEP_RATE.sleep()

