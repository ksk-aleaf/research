#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest("telepabot")
import const

#必要なメッセージファイル
from hark_msgs.msg import HarkSource  # @UnresolvedImport
from hark_msgs.msg import HarkSourceVal  # @UnusedImport @UnresolvedImport


#d_theta = []
locSrcList = [] #harkから得た定位音源情報を整形して入れる配列
#prevLocSrcList = [] #直前の定位音源情報
#vanLocSrcList = [] #消えた音源の情報をしばらく入れる配列
#msg_select = [] #視聴選択範囲内の音源情報
cvCenterImage = None
cvLeftImage = None
cvRightImage = None


harkMsg = HarkSource() #HarkSource型のままのメッセージ
prev_msg = HarkSource()
recogWordList = [] #have list from 0 to VERT_MAX_RESULT_NUM


#音源視聴範囲角度
listenRangeStartAngle = 0
listenRangeEndAngle = 0
listenRangeStartX = 0
listenRangeEndX = 0

#分離音源視聴UI
listenSeparateSoundCount = 0
listenSeparateSoundFlag = False
sendSeparateAngleInfoTime = 0
sendSeparateAngleInfoDuration = 1.0
listenRangeList = []

#定位ソース
harkSource = HarkSource()
#harkSourceInListenRange = HarkSource()

#kobuki
odometry_orien_z = 0
prev_odometry_orien_z = 0
reset_odometry_counter = 0
reset_odometry_flag = False
autoRotateOdometry = 0

#joy stick 入力
joyInput = {const.JOY_FRONT_BACK_LABEL:0.0, const.JOY_LEFT_RIGHT_LABEL:0.0,const.JOY_BUTTON_LABEL:0}
autoRotatingFlag = False
manualRotatingFlag = False
manualRotateStartPeriod = 0
manualRotateDirection = const.JOY_STAY
robotMoveDirection = const.JOY_STAY
autoRotateStartPeriod = 0
autoRotateTimeout = 0