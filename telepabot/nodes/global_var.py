#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest("telepabot")
import const

#必要なメッセージファイル
from hark_msgs.msg import HarkSource  # @UnresolvedImport
from hark_msgs.msg import HarkSourceVal  # @UnusedImport @UnresolvedImport

#GUI動作モード
robotManipulateMode = const.ROBOT_MANIPULATE_FULLAUTO
effectMode = const.EFFECT_OFF

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
# listenRangeStartAngle = 0
# listenRangeEndAngle = 0
# listenRangeStartX = 0
# listenRangeEndX = 0

#分離音源視聴UI
listenSeparateSoundCount = 0
mainListenRange = None
subListenRange = None
#listenSeparateSoundFlag = False
#sendSeparateAngleInfoTime = 0
#sendSeparateAngleInfoDuration = 1.0
listenRangeList = []

#定位ソース
harkSource = HarkSource()
#harkSourceInListenRange = HarkSource()

#kobuki
odometryOrienZ = 0
#odometry_orien_z_no_reset = 0
resetOdometryCounter = 0
autoRotateOdometry = 0 #自動回転のodometry値
odometryOverThresholdCount = 0 #odometryが1またぐ場合のためのカウンタ
#rotateOverOneOdometryCount = 0

#joy stick 入力
joyInput = {const.JOY_FRONT_BACK_LABEL:0.0, const.JOY_LEFT_RIGHT_LABEL:0.0,const.JOY_BUTTON_LABEL:0}
isAutoRotating = False
isManualRotating = False
manualRotateStartPeriod = 0
manualRotateDirection = const.STAY
robotMoveDirection = const.STAY
robotPrevMoveDirection = const.STAY
autoRotateStartPeriod = 0
autoRotateTimeout = 0

#キー入力
rightKeyPressFlag = False
leftKeyPressFlag = False
upKeyPressFlag = False
downKeyPressFlag = False

#マウスクリック
rightClickFlag = False
leftClickFlag = False

#システムステータス(現状、harkの定位・分離が動いているか)
systemStatusFlag = True