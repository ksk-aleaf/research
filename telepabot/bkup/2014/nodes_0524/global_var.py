#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest("telepabot")
import rospy
import copy

#必要なメッセージファイル
from hark_msgs.msg import HarkSource
from hark_msgs.msg import HarkSourceVal
from hark_msgs.msg import HarkSrcWave
from hark_msgs.msg import HarkSrcWaveVal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as SIm
from std_msgs.msg import String
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
from ui_alt.msg import tf_uialt
from sensor_msgs.msg import Joy

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import cv
import threading
import time
import math
import tf
import os


#グローバル変数
#~ debugFlag = True
flag = []
dflag = True
cflag = True
lflag = True
sflag = True
#~ vx = []
#~ vy = []
#~ k1 = []
#~ k2 = []
#~ k3 = []
#~ k4 = []
max_theta = []
min_theta = []
max_power = 0
d_theta = []
#~ id = []
#~ a = []
locSrcList = [] #harkから得た定位音源情報を整形して入れる配列
prevLocSrcList = [] #直前の定位音源情報
vanLocSrcList = [] #消えた音源の情報をしばらく入れる配列
msg_select = [] #視聴選択範囲内の音源情報
cv_image = None
harkMsg = HarkSource() #HarkSource型のままのメッセージ
prev_msg = HarkSource()
recogWordList = [] #have list from 0 to VERT_MAX_RESULT_NUM



#~ leftAutoRotBtnList = []#左自動回転のボタンリスト
#~ rightAutoRotBtnList = []#右自動回転のボタンリスト
#~ leftListenBtnList = []#左分離音声視聴ボタンリスト
#~ rightListenBtnList = []#右分離音声視聴ボタンリスト
#~ onImageLeftListenBtnList = []#画面上・左分離音声視聴ボタンリスト
#~ onImageRightListenBtnList = []#画面上・右分離音声視聴ボタンリスト

#マニュアルボタン
#~ rightRotateButton = None
#~ leftRotateButton = None
#~ forwardButton = None
#~ backButton = None
#~ allListenButton = None

#マニュアルボタンのable/disable管理
#~ manualButtonAbleFlag = True
#~ manualFinDisableTime = 0

#painter
wordPainter = QPainter()

#音源視聴範囲角度
listenRangeStartAngle = 0
listenRangeEndAngle = 0

#分離音源視聴フラグ
listenSeparateSoundFlag = False
sendSeparateAngleInfoTime = 0
sendSeparateAngleInfoDuration = 1.0
