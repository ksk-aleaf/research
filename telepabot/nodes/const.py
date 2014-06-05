#! /usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from hark_msgs.msg import HarkSource
from std_msgs.msg import Bool

#openCV import
from cv_bridge import CvBridge, CvBridgeError

#PyQt import
from PyQt4.QtCore import *
from PyQt4.QtGui import *


#SYSTEM NAME
SYSTEM_NAME = "telepabot"

#TOPIC NAME
CENTER_CAM_IMG_TOPIC_NAME = "/usb_cam_center/image_raw"
#SIDE_CAM_IMG_TOPIC_NAME = "/usb_cam/side_image_raw"

#CAM PARAMETER
CAM_DEFAULT_FPS = 30

#カメラ映像の大きさと画面内の表示位置
#CAM_IMG_WID = 640
CAM_WHOLE_IMG_WID = 2560
CAM_CENTER_IMG_WID = 1280
CAM_LEFT_IMG_WID = 640
CAM_RIGHT_IMG_WID = 640
CAM_IMG_HALF_WID = CAM_WHOLE_IMG_WID /2
CAM_IMG_HT = 480
CAM_IMG_HALF_HT = CAM_IMG_HT / 2
CAM_IMG_OFS_X = 0
CAM_IMG_OFS_Y = 200

#cam draw position
CAM_IMG_DRAW_POINT = QPoint(CAM_IMG_OFS_X, CAM_IMG_OFS_Y)

#sec to milsec
MSEC_ONE_SEC = 1000

#central widget fps
CENTRAL_WIDGET_FPS = 30

#central widget draw HZ
CENTRAL_WIDGET_DRAW_HZ = MSEC_ONE_SEC / CENTRAL_WIDGET_FPS

#camera draw HZ
CAM_DRAW_HZ = MSEC_ONE_SEC / CAM_DEFAULT_FPS

#画像内の角度領域
IMG_HOR_VIEW_AGL = 360
IMG_HOR_HALF_VIEW_AGL = float(IMG_HOR_VIEW_AGL) / 2
IMG_ELEV_VIEW_AGL = 45
IMG_ELEV_HALF_VIEW_AGL = float(IMG_ELEV_VIEW_AGL) / 2

#THETA IMG
PIXEL_PER_AZIMUTH = int(CAM_WHOLE_IMG_WID / IMG_HOR_VIEW_AGL)

#定数(言語仕様では定数は無く、書き換え可能なので注意)
#ウィンドウの大きさ
WIN_WID = CAM_WHOLE_IMG_WID + CAM_IMG_OFS_X*2
WIN_HT = CAM_IMG_HT + CAM_IMG_OFS_Y



##################画像データ################################

#消えた音源ソース情報を保持する秒数
VAN_SRC_KEEP_SEC= 3.0

#staticに使いたいインスタンス(今のところコマンド送信用)
L_ROT_CMD = Twist()
L_ROT_CMD.angular.z = 0.88
R_ROT_CMD = Twist()
R_ROT_CMD.angular.z = -0.88
FWD_CMD = Twist()
FWD_CMD.linear.x = 0.1
BACK_CMD = Twist()
BACK_CMD.linear.x = -0.1
#CMD_PUBLISHER = rospy.Publisher('/cmd_vel', Twist)

#image convert static instance
CV_BRIDGE = CvBridge()

#turtlebotにタイムアウトを設定する用
SET_TO_STR = "rosparam set /turtlebot_node/cmd_vel_timeout "
MAN_ROT_TO = 0.4
KEY_MAN_ROT_TO = 0.5
JOY_MAN_ROT_TO = 0.4
DEFAULT_TO = 0.0
TO_PER_THETA = 0.022222
SEND_CMD_OVERHEAD = 1.0

#音源パワー分類用
WEAK_POW_CODE = 0
MEDIUM_POW_CODE = 1
STRONG_POW_CODE = 2
#STRONG_POW_THR = 33.6
#MEDIUM_POW_THR = 33.3
STRONG_POW_THR = 0.5
MEDIUM_POW_THR = 1

#音源視聴範囲設定ID
LISTENABLE = 1
UNLISTENABLE = 0

#視聴範囲指定時にこの幅以下で指定されたら無視する
IGNOR_PIX_THR = 20

#recognize result draw
RECOG_WORD_HEIGHT_RANGE = 20
#RECOG_WORD_DEFAULT_HEIGHT = 100
#MAX_RESULT_NUM = 100

VERT_WORD_MGN = 10
CAM_WORD_MGN = 10
VERT_MAX_WORD_NUM = int((CAM_IMG_OFS_Y - CAM_WORD_MGN) / (RECOG_WORD_HEIGHT_RANGE + VERT_WORD_MGN))
RECOG_DRAW_BTM_YAXIS = CAM_IMG_OFS_Y - CAM_WORD_MGN
WIDTH_PER_CHAR = 19
HEIGHT_PER_CHAR = 20

#topic publisher
CLASSIFIED_SOURCE_PUB = rospy.Publisher("ClassifiedSource", HarkSource)
SELECTED_SOURCE_PUB = rospy.Publisher("SelectedSource", HarkSource)
SEP_LIS_TRIG_PUB = rospy.Publisher("SeparateListenTrigger", Bool)

#topic name
HARK_JULIUS_SOURCE_TOPIC_NAME = "HarkJuliusSrc"
HARK_LOC_SOURCE_TOPIC_NAME = "HarkSource"
#JULIUS_RECOG_RESULT_TOPIC_NAME = "RecogResult"


#英単語略称一覧
"""
ARW ARROW
ROT ROTATE
BTN BUTTON
L LEFT
R RIGHT
MAN MANUAL(手動)
BND BOUNDARY
FWD FORWARD
LIS LISTEN
MARGIN MGN
OFFSET OFS
TIMEOUT TO
RECOG RECOGNITION
"""
