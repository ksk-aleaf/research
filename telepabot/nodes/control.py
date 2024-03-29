#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('ui_alt')
import rospy

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

import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
import cv
import threading
import time
import math
import tf
import os

#グローバル変数
flag = []
dflag = False
cflag = False
lflag = False
sflag = False
vx = []
vy = []
k1 = []
k2 = []
k3 = []
k4 = []
max_theta = []
min_theta = []
max_power = 0
d_theta = []
id = []
xarray = []
yarray = []
a = []
source = []
msg_select = []
cv_image = None
tfframe = []
prev_msg = HarkSource()

FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'right_shoulder'
        ]

def localization_callback(data):

    global source
    global flagoos
    global max_power
    global prev_msg

    source = []
    flagoos = False

    pub = rospy.Publisher('DetectedSource', HarkSource)
    msg = HarkSource()

    for i in range(len(data.src)):#data[]の中身をsource[]に移し、powerが今までの最大値より大きければmax_powerを更新
        
        if(data.src[i].theta >= -45 and data.src[i].theta <= 45):
            x = 320*(1-(math.tan(math.radians(data.src[i].theta))/math.tan(math.radians(28.5))))
        else:
            x = -100
            flagoos = True

        array = [data.src[i].id, x, data.src[i].theta, data.src[i].power]
        source.append(array)

        max_power = source[0][3]
        for j in range(len(source)):
            if source[j][3] - max_power > 0:
                max_power = source[j][3]

    for k in range(len(data.src)):#角度ごとにメッセージIDを付与
        msg_val = HarkSourceVal()
        if(data.src[k].theta >= -10 and data.src[k].theta < 10):
            msg_val.id = 0
        elif(data.src[k].theta >= -30 and data.src[k].theta < -10):
            msg_val.id = 1
        elif(data.src[k].theta >= 10 and data.src[k].theta <= 30):
            msg_val.id = 2
        elif(data.src[k].theta > -75 and data.src[k].theta < -30):
            msg_val.id = 3
        elif(data.src[k].theta > -105 and data.src[k].theta <= -75):
            msg_val.id = 4
        elif(data.src[k].theta > -135 and data.src[k].theta <= -105):
            msg_val.id = 5
        elif(data.src[k].theta > -165 and data.src[k].theta <= -135):
            msg_val.id = 6
        elif(data.src[k].theta > 30 and data.src[k].theta <= 75):
            msg_val.id = 7
        elif(data.src[k].theta > 75 and data.src[k].theta <= 105):
            msg_val.id = 8
        elif(data.src[k].theta > 105 and data.src[k].theta <= 135):
            msg_val.id = 9
        elif(data.src[k].theta > 135 and data.src[k].theta <= 165):
            msg_val.id = 10
        elif((data.src[k].theta > 165 and data.src[k].theta <= 180) or (data.src[k].theta >= -180 and data.src[k].theta <= -165)):
            msg_val.id = 11
        msg_val.x  = data.src[k].x
        msg_val.y  = data.src[k].y
        msg_val.theta = data.src[k].theta
        msg.src.append(msg_val)
        msg.exist_src_num += 1
    
    for l in range(len(prev_msg.src)):#直前のメッセージソースと比較して、IDが異なるものがあればメッセージソースに追加
        flag = False
        for n in range(len(msg.src)):
            if(prev_msg.src[l].id == msg.src[n].id):
                flag = True
        if(flag == False):
            msg.src.append(prev_msg.src[l])
            msg.exist_src_num += 1
    pub.publish(msg)
    prev_msg = msg

class ControlWidget(QtGui.QWidget):#GUIのクラス

    def __init__(self, parent = None):#画面上のオブジェクトを初期化
        super(ControlWidget, self).__init__(parent)
        
        self.setMinimumSize(500, 500)
        self.setMaximumSize(500, 500)
        self.setWindowTitle("Control Panel")
        self.frame = QtGui.QFrame(self)
        self.frame.setFrameStyle(QtGui.QFrame.Box | QtGui.QFrame.Raised)
        self.frame.setLineWidth(2)
        self.frame.setGeometry(0, 0, 500, 500)
        self.label = QtGui.QLabel(self)
        self.label.setPixmap(QtGui.QPixmap("/home/user/ros/ui_alt/icon/turtlebot.png"))
        self.label.setGeometry(226, 224, 64, 64)

        self.hicon = QtGui.QIcon(self)
        self.ricon = QtGui.QIcon(self)
        self.hicon.addFile("/home/user/ros/ui_alt/icon/3076.png")
        self.ricon.addFile("/home/user/ros/ui_alt/icon/turtlebot_logo.png")
        
        self.rbuttonn30 = QtGui.QPushButton(self.ricon, "", self)
        self.rbuttonn30.setGeometry(340, 54, 30, 30)
        self.rbuttonn30.clicked.connect(self.moven30)
        self.hbuttonn60 = QtGui.QPushButton(self.hicon, "", self)
        self.rbuttonn60 = QtGui.QPushButton(self.ricon, "", self)
        self.hbuttonn60.setGeometry(382, 150, 30, 30)
        self.rbuttonn60.setGeometry(416, 130, 30, 30)
        self.hbuttonn60.clicked.connect(self.listenn60)
        self.rbuttonn60.clicked.connect(self.moven60)
        self.hbuttonn90 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn90 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbuttonn90.setGeometry(405, 235, 30, 30)
        self.rbuttonn90.setGeometry(445, 235, 30, 30)
        self.hbuttonn90.clicked.connect(self.listenn90)
        self.rbuttonn90.clicked.connect(self.moven90)
        self.hbuttonn120 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn120 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbuttonn120.setGeometry(382, 320, 30, 30)
        self.rbuttonn120.setGeometry(416, 340, 30, 30)
        self.hbuttonn120.clicked.connect(self.listenn120)
        self.rbuttonn120.clicked.connect(self.moven120)
        self.hbuttonn150 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn150 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbuttonn150.setGeometry(320, 382, 30, 30)
        self.rbuttonn150.setGeometry(340, 416, 30, 30)
        self.hbuttonn150.clicked.connect(self.listenn150)
        self.rbuttonn150.clicked.connect(self.moven150)

        self.rbutton30 = QtGui.QPushButton(self.ricon, "", self)
        self.rbutton30.setGeometry(130, 54, 30, 30)
        self.rbutton30.clicked.connect(self.move30)
        self.hbutton60 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton60 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbutton60.setGeometry(88, 150, 30, 30)
        self.rbutton60.setGeometry(56, 130, 30, 30)
        self.hbutton60.clicked.connect(self.listen60)
        self.rbutton60.clicked.connect(self.move60)
        self.hbutton90 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton90 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbutton90.setGeometry(65, 235, 30, 30)
        self.rbutton90.setGeometry(25, 235, 30, 30)
        self.hbutton90.clicked.connect(self.listen90)
        self.rbutton90.clicked.connect(self.move90)
        self.hbutton120 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton120 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbutton120.setGeometry(88, 320, 30, 30)
        self.rbutton120.setGeometry(56, 340, 30, 30)
        self.hbutton120.clicked.connect(self.listen120)
        self.rbutton120.clicked.connect(self.move120)
        self.hbutton150 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton150= QtGui.QPushButton(self.ricon, "" ,self)
        self.hbutton150.setGeometry(150, 382, 30, 30)
        self.rbutton150.setGeometry(130, 416, 30, 30)
        self.hbutton150.clicked.connect(self.listen150)
        self.rbutton150.clicked.connect(self.move150)

        self.hbutton180 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton180 = QtGui.QPushButton(self.ricon, "" ,self)
        self.hbutton180.setGeometry(235, 405, 30, 30)
        self.rbutton180.setGeometry(235, 445, 30, 30)
        self.hbutton180.clicked.connect(self.listen180)
        self.rbutton180.clicked.connect(self.move180)

    def paintEvent(self, event):

        global source
        global max_power
        global lflag
        
		#描画用のインスタンスを生成？
        path1_1 = QtGui.QPainterPath()
        path1_2 = QtGui.QPainterPath()
        path1_3 = QtGui.QPainterPath()
        path2_1 = QtGui.QPainterPath()
        path2_2 = QtGui.QPainterPath()
        path2_3 = QtGui.QPainterPath()
        path3_1 = QtGui.QPainterPath()
        path3_2 = QtGui.QPainterPath()
        path3_3 = QtGui.QPainterPath()
        path4_1 = QtGui.QPainterPath()
        path4_2 = QtGui.QPainterPath()
        path4_3 = QtGui.QPainterPath()
        path5_1 = QtGui.QPainterPath()
        path5_2 = QtGui.QPainterPath()
        path5_3 = QtGui.QPainterPath()
        path6_1 = QtGui.QPainterPath()
        path6_2 = QtGui.QPainterPath()
        path6_3 = QtGui.QPainterPath()
        path7_1 = QtGui.QPainterPath()
        path7_2 = QtGui.QPainterPath()
        path7_3 = QtGui.QPainterPath()
        path8_1 = QtGui.QPainterPath()
        path8_2 = QtGui.QPainterPath()
        path8_3 = QtGui.QPainterPath()
        path9_1 = QtGui.QPainterPath()
        path9_2 = QtGui.QPainterPath()
        path9_3 = QtGui.QPainterPath()

        p1= QtGui.QPainter(self)
        
        pen1 = QtGui.QPen(QtCore.Qt.green, 4, QtCore.Qt.SolidLine)
        pen2 = QtGui.QPen(QtCore.Qt.yellow, 4, QtCore.Qt.SolidLine)
        pen3 = QtGui.QPen(QtCore.Qt.red, 4, QtCore.Qt.SolidLine)
        
        for i in range(len(source)):

            if(source[i][2] > -75 and source[i][2] < -30):
                
                power = source[i][3]

                path1_1.moveTo(378, 171)
                path1_2.moveTo(370, 169)
                path1_3.moveTo(364, 167)                
                path1_1.cubicTo(374, 173, 379, 182, 383, 180)#基準点2つの中点を通るよう、
                path1_2.cubicTo(362, 174, 372, 192, 380, 187)
                path1_3.cubicTo(351, 175, 366, 201, 379, 193)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path1_1)
                    p1.drawPath(path1_2)
                    p1.drawPath(path1_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path1_1)
                    p1.drawPath(path1_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path1_1)
                   
            if(source[i][2] > -105 and source[i][2] <= -75):

                power = source[i][3]

                path2_1.moveTo(400, 245)
                path2_2.moveTo(395, 240)
                path2_3.moveTo(390, 235)                
                path2_1.cubicTo(395, 245, 395, 255, 400, 255)
                path2_2.cubicTo(385, 240, 385, 260, 395, 260)
                path2_3.cubicTo(375, 235, 375, 265, 390, 265)

                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path2_1)
                    p1.drawPath(path2_2)
                    p1.drawPath(path2_3)

                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path2_1)
                    p1.drawPath(path2_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path2_1)
                    
            if(source[i][2] > -135 and source[i][2] <= -105):

                power = source[i][3]

                path3_1.moveTo(378, 329)
                path3_2.moveTo(370, 331)
                path3_3.moveTo(364, 333)                
                path3_1.cubicTo(374, 327, 379, 318, 383, 320)
                path3_2.cubicTo(362, 326, 372, 308, 380, 313)
                path3_3.cubicTo(351, 325, 366, 299, 379, 307)

                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path3_1)
                    p1.drawPath(path3_2)
                    p1.drawPath(path3_3)

                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path3_1)
                    p1.drawPath(path3_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path3_1)
                    
            if(source[i][2] > -165 and source[i][2] <= -135):
                
                power = source[i][3]
                
                path4_1.moveTo(329, 378)
                path4_2.moveTo(331, 370)
                path4_3.moveTo(333, 364)                
                path4_1.cubicTo(327, 374, 318, 379, 320, 383)
                path4_2.cubicTo(326, 362, 308, 372, 313, 380)
                path4_3.cubicTo(325, 351, 299, 366, 307, 379)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path4_1)
                    p1.drawPath(path4_2)
                    p1.drawPath(path4_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path4_1)
                    p1.drawPath(path4_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path4_1)
                    
            if((source[i][2] >= -180 and source[i][2] <= -180) or (source[i][2] > 165 and source[i][2] <= 180)):
                
                power = source[i][3]

                path5_1.moveTo(245, 400)
                path5_2.moveTo(240, 395)
                path5_3.moveTo(235, 390)                
                path5_1.cubicTo(245, 395, 255, 395, 255, 400)
                path5_2.cubicTo(240, 385, 260, 385, 260, 395)
                path5_3.cubicTo(235, 375, 265, 375, 265, 390)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path5_1)
                    p1.drawPath(path5_2)
                    p1.drawPath(path5_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path5_1)
                    p1.drawPath(path5_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path5_1)

            if(source[i][2] > 135 and source[i][2] <= 165):
                
                power = source[i][3]

                path6_1.moveTo(171, 378)
                path6_2.moveTo(169, 370)
                path6_3.moveTo(167, 364)                
                path6_1.cubicTo(173, 374, 182, 379, 180, 383)
                path6_2.cubicTo(174, 362, 192, 372, 187, 380)
                path6_3.cubicTo(175, 351, 201, 366, 193, 379)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path6_1)
                    p1.drawPath(path6_2)
                    p1.drawPath(path6_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path6_1)
                    p1.drawPath(path6_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path6_1)
                    

            if(source[i][2] > 105 and source[i][2] <= 135):
                
                power = source[i][3]

                path7_1.moveTo(122, 329)
                path7_2.moveTo(130, 331)
                path7_3.moveTo(136, 333)                
                path7_1.cubicTo(126, 327, 121, 318, 117, 320)
                path7_2.cubicTo(138, 326, 128, 308, 120, 313)
                path7_3.cubicTo(149, 325, 134, 299, 121, 307)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path7_1)
                    p1.drawPath(path7_2)
                    p1.drawPath(path7_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path7_1)
                    p1.drawPath(path7_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path7_1)

            if(source[i][2] > 75 and source[i][2] <= 105):
                
                power = source[i][3]

                path8_1.moveTo(100, 245)
                path8_2.moveTo(105, 240)
                path8_3.moveTo(110, 235)                
                path8_1.cubicTo(105, 245, 105, 255, 100, 255)
                path8_2.cubicTo(115, 240, 115, 260, 105, 260)
                path8_3.cubicTo(125, 235, 125, 265, 110, 265)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path8_1)
                    p1.drawPath(path8_2)
                    p1.drawPath(path8_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path8_1)
                    p1.drawPath(path8_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path8_1)


            if(source[i][2] > 30  and source[i][2] <= 75):
                
                power = source[i][3]

                path9_1.moveTo(122, 171)
                path9_2.moveTo(130, 169)
                path9_3.moveTo(136, 167)                
                path9_1.cubicTo(126, 173, 121, 182, 117, 180)
                path9_2.cubicTo(138, 174, 128, 192, 120, 187)
                path9_3.cubicTo(149, 175, 134, 201, 121, 193)
                
                if(max_power - power == 0):
                    p1.setPen(pen3)
                    p1.drawPath(path9_1)
                    p1.drawPath(path9_2)
                    p1.drawPath(path9_3)
                    
                if(max_power - power > 0 and max_power - power <= 1):
                    p1.setPen(pen2)
                    p1.drawPath(path9_1)
                    p1.drawPath(path9_2)     
                    
                if(max_power - power > 1):
                    p1.setPen(pen1)
                    p1.drawPath(path9_1)

        p2 = QtGui.QPainter(self)
        pen4 = QtGui.QPen(QtCore.Qt.gray, 2, QtCore.Qt.DotLine)
        p2.setPen(pen4)
        p2.drawArc(80, 80, 340, 340, 120*16, 300*16)
        p2.drawArc(40, 40, 420, 420, 120*16, 300*16)
        p2.drawLine(250, 250, 460, 250)
        p2.drawLine(250, 250, 40, 250)
        p2.drawLine(250, 250, 250, 460)
        p2.drawLine(250, 250, 436, 145)
        p2.drawLine(250, 250, 64, 145)
        p2.drawLine(250, 250, 145, 436)
        p2.drawLine(250, 250, 436, 355)
        p2.drawLine(250, 250, 64, 355)
        p2.drawLine(250, 250, 355, 436)

        p3 = QtGui.QPainter(self)
        pen5 = QtGui.QPen(QtCore.Qt.magenta, 3, QtCore.Qt.SolidLine)
        p3.setPen(pen5)
        brush = QtGui.QBrush(QtCore.Qt.magenta, QtCore.Qt.DiagCrossPattern)
        p3.setBrush(brush)
        p3.drawPie(40, 40, 420, 420, 60*16, 60*16)
        
        self.update()
        time.sleep(0.0333)

    def moven30(self):#-30度
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -0.88#角速度？
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.66666")#タイムアウトの時間を指定するとその時間分回るので角度が指定できる
        pub.publish(cmd)
        rospy.loginfo(cmd)

    def moven60(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 1.33333")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def moven90(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 2.0")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def moven120(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 2.66666")
        pub.publish(cmd)
        rospy.loginfo(cmd)

    def moven150(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 3.33333")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def move30(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.66666")
        pub.publish(cmd)
        rospy.loginfo(cmd)

    def move60(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 1.33333")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def move90(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 2.0")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def move120(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 2.66666")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def move150(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 3.33333")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def move180(self):
        
        global source

        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = 0.88
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 4.0")
        pub.publish(cmd)
        rospy.loginfo(cmd) 

    def listenn60(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] < -30 and source[i][2] > -75):
                
                append = HarkSourceVal()
                #append.id = 3
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)
    
    def listenn90(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()#音源定位用のインスタンス生成
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] <= -75 and source[i][2] > -105):
                
                append = HarkSourceVal()
                #append.id = 4
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)
        
    def listenn120(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] <= -105 and source[i][2] > -135):
                
                append = HarkSourceVal()
                #append.id = 5
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)

    def listenn150(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] <= -135 and source[i][2] > -165):
                
                append = HarkSourceVal()
                #append.id = 6
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()#3hzのレートなので1/3杪間スリープ
        trigger.publish(select)

    def listen60(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] > 30 and source[i][2] <= 75):
                
                append = HarkSourceVal()
                #append.id = 7
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)
    
    def listen90(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] > 75 and source[i][2] <= 105):
                
                append = HarkSourceVal()
                #append.id = 8
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)
        
    def listen120(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] > 105 and source[i][2] <= 135):
                
                append = HarkSourceVal()
                #append.id = 9
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)

    def listen150(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if(source[i][2] > 135 and source[i][2] <= 165):
                
                append = HarkSourceVal()
                #append.id = 10
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)

    def listen180(self):
        
        global source
        global sflag
        global msg_select
        
        msg_select = HarkSource()
        msg_select.exist_src_num = 0
        trigger = rospy.Publisher('Trigger', Bool)
        pub_select = rospy.Publisher('SourceSelect', HarkSource)
        select = False
        flag = False
        
        for i in range(len(source)):

            if((source[i][2] <= -165 and source[i][2] >= -180) or (source[i][2] > 165 and source[i][2] <= 180)):
                
                append = HarkSourceVal()
                #append.id = 11
                append.id = 0
                msg_select.src.append(append)
                flag = True
                msg_select.exist_src_num = len(source)
                select = True

        pub_select.publish(msg_select)
        print msg_select
        r = rospy.Rate(3)
        r.sleep()
        trigger.publish(select)

def subscriber():
    
    rospy.init_node('UIALT', anonymous = True)
    rospy.Subscriber('HarkSource', HarkSource, localization_callback, buff_size = 1)
    rospy.spin()#rospyのサービスまたはノードがシャットダウンされるまで待機？

class QtThread():
    def main():
        app = QtGui.QApplication(sys.argv)
        cw = ControlWidget()
        cw.show()
        app.exec_()

    t = threading.Thread(None, main, 'Qt')
    t.setDaemon(1)
    t.start()

if __name__ == '__main__':

    t = QtThread()
    subscriber()
