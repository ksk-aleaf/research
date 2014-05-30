#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('openni_uialt')
import rospy

#必要なメッセージファイル
#from hark_msgs.msg import HarkSource
#from hark_msgs.msg import HarkSourceVal
#from hark_msgs.msg import HarkSrcWave
#from hark_msgs.msg import HarkSrcWaveVal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image as SIm
from std_msgs.msg import String
from std_msgs.msg import Bool
#from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
#from ui_alt.msg import tf_uialt

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
#prev_msg = HarkSource()

FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'right_shoulder'
        ]

class ControlWidget(QtGui.QWidget):

    def __init__(self, parent = None):
        super(ControlWidget, self).__init__(parent)
        
        self.setMinimumSize(500, 500)
        self.setMaximumSize(500, 500)
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
        
        #self.hbuttonn60 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn60 = QtGui.QPushButton(self.ricon, "" ,self)
        #self.hbuttonn60.setGeometry(382, 150, 30, 30)
        self.rbuttonn60.setGeometry(416, 130, 30, 30)
        self.rbuttonn60.clicked.connect(self.moven60)
        #self.hbuttonn60.clicked.connect(self.listenn60)
        #self.hbuttonn90 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn90 = QtGui.QPushButton(self.ricon, "" ,self)
        #self.hbuttonn90.setGeometry(405, 235, 30, 30)
        self.rbuttonn90.setGeometry(445, 235, 30, 30)
        self.rbuttonn90.clicked.connect(self.moven90)
        #self.hbuttonn90.clicked.connect(self.listenn90)
        #self.hbuttonn120 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn120 = QtGui.QPushButton(self.ricon, "" ,self)
        #self.hbuttonn120.setGeometry(382, 320, 30, 30)
        self.rbuttonn120.setGeometry(416, 340, 30, 30)
        self.rbuttonn120.clicked.connect(self.moven120)
        #self.hbuttonn120.clicked.connect(self.listenn120)
        #self.hbuttonn150 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbuttonn150 = QtGui.QPushButton(self.ricon, "" ,self)
        #self.hbuttonn150.setGeometry(320, 382, 30, 30)
        self.rbuttonn150.setGeometry(340, 416, 30, 30)
        #self.hbuttonn150.clicked.connect(self.listenn150)

        #self.hbutton180 = QtGui.QPushButton(self.hicon, "" ,self)
        self.rbutton180 = QtGui.QPushButton(self.ricon, "" ,self)
        #self.hbutton180.setGeometry(235, 405, 30, 30)
        self.rbutton180.setGeometry(235, 445, 30, 30)
        #self.hbutton180.clicked.connect(self.listen180)
        
    def moven60(self):

        rospy.init_node('command')
        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -1.76
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 0.6666666666666666666666666")
        pub.publish(cmd)
        print ('published')

        #rospy.loginfo(cmd)

    def moven90(self):

        rospy.init_node('command')
        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)

        cmd.angular.z = -1.76
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 1.00000000000000000000000000")
        pub.publish(cmd)
        print ('published')

    def moven120(self):
        
        rospy.init_node('command')
        cmd = Twist()
        pub = rospy.Publisher('/cmd_vel', Twist)


        cmd.angular.z = -1.76
        os.system("rosparam set /turtlebot_node/cmd_vel_timeout 1.3333333333333333333333333")
        pub.publish(cmd)
        print('published')

    def paintEvent(self, event):

        global source
        global lflag
        
        path1_1 = QtGui.QPainterPath()
        path1_2 = QtGui.QPainterPath()
        path1_3 = QtGui.QPainterPath()
        path1_1.moveTo(400, 245)
        path1_2.moveTo(395, 240)
        path1_3.moveTo(390, 235)
        path1_1.cubicTo(395, 245, 395, 255, 400, 255)
        path1_2.cubicTo(385, 240, 385, 260, 395, 260)
        path1_3.cubicTo(375, 235, 375, 265, 390, 265)

        p1 = QtGui.QPainter(self)
        pen1 = QtGui.QPen(QtCore.Qt.red, 3, QtCore.Qt.SolidLine)
        p1.setPen(pen1)
        
        p2 = QtGui.QPainter(self)
        pen2 = QtGui.QPen(QtCore.Qt.cyan, 3, QtCore.Qt.DotLine)
        p2.setPen(pen2)
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
        pen3 = QtGui.QPen(QtCore.Qt.yellow, 3, QtCore.Qt.SolidLine)
        p3.setPen(pen3)
        brush = QtGui.QBrush(QtCore.Qt.yellow, QtCore.Qt.DiagCrossPattern)
        p3.setBrush(brush)
        p3.drawPie(40, 40, 420, 420, 60*16, 60*16)
        
        self.update()
        time.sleep(0.0333)

def main():
    app = QtGui.QApplication(sys.argv)
    cw = ControlWidget()
    cw.show()
    app.exec_()

if __name__ == '__main__':

    main()
