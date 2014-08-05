#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest("telepabot")
import const

#必要なメッセージファイル
from hark_msgs.msg import HarkSource  # @UnresolvedImport
from hark_msgs.msg import HarkSourceVal  # @UnusedImport @UnresolvedImport


#グローバル変数
flag = []
dflag = True
cflag = True
lflag = True
sflag = True
max_theta = []
min_theta = []
max_power = 0
d_theta = []
locSrcList = [] #harkから得た定位音源情報を整形して入れる配列
prevLocSrcList = [] #直前の定位音源情報
vanLocSrcList = [] #消えた音源の情報をしばらく入れる配列
msg_select = [] #視聴選択範囲内の音源情報
cvCenterImage = None
cvLeftImage = None
cvRightImage = None

#cvImage
harkMsg = HarkSource() #HarkSource型のままのメッセージ
prev_msg = HarkSource()
recogWordList = [] #have list from 0 to VERT_MAX_RESULT_NUM


#音源視聴範囲角度
listenRangeStartAngle = 0
listenRangeEndAngle = 0

#分離音源視聴フラグ
listenSeparateSoundFlag = False
sendSeparateAngleInfoTime = 0
sendSeparateAngleInfoDuration = 1.0

#定位ソース
harkSource = HarkSource()
msg_select_gl = HarkSource()

#joy stick 入力
joyInput = {const.JOY_FRONT_BACK_LABEL:0.0, const.JOY_LEFT_RIGHT_LABEL:0.0,const.JOY_BUTTON_LABEL:0}
ifAutoRotate = False