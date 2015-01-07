#!/usr/bin/python
# -*-coding: utf-8 -*-
import socket
import struct

#pyqt
import sys
from PyQt4 import QtGui
from PyQt4 import QtCore

#ros
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import MultiArrayLayout

#WINDOW
WIN_X_POS = 100
WIN_Y_POS = 100
WIN_WID = 200
WIN_HT = 200
SYSTEM_NAME = "GUI MODE CHANGER"

#ボタン
CHANGE_GUI_MODE_BTN_WID = 120
CHANGE_GUI_MODE_BTN_HT = 20
CHANGE_GUI_MODE_BTN_Y = 20
CHANGE_GUI_MODE1_BTN_X = 20
CHANGE_GUI_MODE2_BTN_X = CHANGE_GUI_MODE1_BTN_X + CHANGE_GUI_MODE_BTN_WID + 20
CHANGE_GUI_MODE3_BTN_X = CHANGE_GUI_MODE2_BTN_X + CHANGE_GUI_MODE_BTN_WID + 20
GUI_MODE1_LABEL = "GUI MODE 1"
GUI_MODE2_LABEL = "GUI MODE 2"
GUI_MODE3_LABEL = "GUI MODE 3"


#GUIモード
ROBOT_MANIPULATE_AUTO = 0
ROBOT_MANIPULATE_MANUAL = 1
EFFECT_ON = 1
EFFECT_OFF = 0
ROBOT_MANIPULATE_INDEX = 0
EFFECT_INDEX = 1

#ROS
MAIN_NODE_NAME = "GuiMode"
ROBOT_MANIPULATE_NODE_NAME = "RobotManipulateMode"
EFFECT_NODE_NAME = "EffectMode"

#グローバル変数
#guiMode = [ROBOT_MANIPULATE_AUTO,EFFECT_ON]
robotManipulateMode = ROBOT_MANIPULATE_AUTO
effectMode = EFFECT_ON
robot_publisher = rospy.Publisher(ROBOT_MANIPULATE_NODE_NAME, UInt8, queue_size=10)
effect_publisher = rospy.Publisher(EFFECT_NODE_NAME, UInt8, queue_size=10)


class MainWindow(QtGui.QWidget):
	def __init__(self):
		super(MainWindow, self).__init__()
		self.initBtn()
		rospy.init_node(MAIN_NODE_NAME, anonymous=True)
		#rospy.init_node(EFFECT_NODE_NAME, anonymous=True)
	
	def initBtn(self):
		ChangeGuiModeButton(self,ROBOT_MANIPULATE_AUTO,EFFECT_ON,CHANGE_GUI_MODE1_BTN_X,CHANGE_GUI_MODE_BTN_Y,GUI_MODE1_LABEL)
		ChangeGuiModeButton(self,ROBOT_MANIPULATE_AUTO,EFFECT_OFF,CHANGE_GUI_MODE2_BTN_X,CHANGE_GUI_MODE_BTN_Y,GUI_MODE2_LABEL)
		ChangeGuiModeButton(self,ROBOT_MANIPULATE_MANUAL,EFFECT_ON,CHANGE_GUI_MODE3_BTN_X,CHANGE_GUI_MODE_BTN_Y,GUI_MODE3_LABEL)

class ChangeGuiModeButton(QtGui.QPushButton):
	def __init__(self,parent,robotManipulateMode,effectMode,xPos,yPos,btnText):
		QtGui.QPushButton.__init__(self,parent)
		#self.guiMode = [robotManipulateMode,effectMode]
		self.robotManipulateMode = robotManipulateMode
		self.effectMode = effectMode
		self.xPos = xPos
		self.yPos = yPos
		self.setText(btnText)
		self.setGeometry(self.xPos,self.yPos,CHANGE_GUI_MODE_BTN_WID,CHANGE_GUI_MODE_BTN_HT)
		self.clicked.connect(self.changeMode)
		self.show()
	
	def changeMode(self):
		global robotManipulateMode,effectMode,publisher
		#guiMode = self.guiMode
		
# 		dim0 = MultiArrayDimension()
# 		dim1 = MultiArrayDimension()
# 		dim0.label = "height"
# 		dim0.size = 1*2
# 		dim0.stride = 2
# 		dim1.label = "width"
# 		dim1.size = 2
# 		dim1.stride = 2
# 		
# 		dim = [dim0,dim1]
# 		
# 		layout = MultiArrayLayout()
# 		layout.dim = dim
# 		layout.data_offset = 0
# 		
# 		msg = UInt8MultiArray()
# 		msg.data = guiMode
# 		msg.layout = layout
# 		
# 		print msg
# 		guiMode[ROBOT_MANIPULATE_INDEX] = self.robotManipulateMode
# 		guiMode[EFFECT_INDEX] = self.effectMode
		robotManipulateMode = self.robotManipulateMode
		effectMode = self.effectMode
		robot_publisher.publish(robotManipulateMode)
		effect_publisher.publish(effectMode)

def initialize():
	app = QtGui.QApplication(sys.argv)
	window = MainWindow()
	window.setWindowTitle(SYSTEM_NAME)
	window.setGeometry(WIN_X_POS, WIN_Y_POS, WIN_WID, WIN_HT)
	window.show()
	return app,window

if __name__ == '__main__':
	app,window = initialize()
	app.exec_()




