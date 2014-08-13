#!/usr/bin/python
# -*-coding: utf-8 -*-
import socket
import struct

#pyqt
import sys
from PyQt4 import QtGui
from PyQt4 import QtCore

HOST = 'localhost'	# The remote host
PORT = 9999		   # The same port as used by the server

#HARK LOCALIZE PARAM
NUM_SOURCE = 3 #int index:0
MIN_DEG = -180 #int index:1
MAX_DEG = 180 #int index:2
LOWER_BOUND_FREQUENCY = 500 #int index:3
UPPER_BOUND_FREQUENCY = 2800 #int index:4
THRESH = 36 #float index:5
PAUSE_LENGTH = 800 # float index:6
MIN_SRC_INTERVAL = 20 # float index:7
MIN_TFINDEX_INTERVAL = 6.0 # float index:8
COMPARE_MODE = 0.0 #index:9

#HARK SEPARATION PARAM
LX = 0.85 # float index:10
TIME_CONSTANT = 16000 # int index:11

#CONFIG
CONFIG_PARAMS = [NUM_SOURCE,MIN_DEG, MAX_DEG, LOWER_BOUND_FREQUENCY, UPPER_BOUND_FREQUENCY, THRESH, PAUSE_LENGTH, MIN_SRC_INTERVAL, MIN_TFINDEX_INTERVAL, COMPARE_MODE, LX, TIME_CONSTANT]
CONFIG_PARAMS_NAME = ["NUM_SOURCE","MIN_DEG", "MAX_DEG", "LOWER_BOUND_FREQUENCY", "UPPER_BOUND_FREQUENCY", "THRESH", "PAUSE_LENGTH", "MIN_SRC_INTERVAL", "MIN_TFINDEX_INTERVAL", "COMPARE_MODE", "LX", "TIME_CONSTANT"]
CONFIG_PARAM_INFO = []
CONFIG_PARAM_NUM = 4
PARAM_BOXES = []
HT_PER_PARAM = 40



#CHANGE PARAM BUTTON
ROUGH_PLUS_BTN_LABEL = "->>"
PLUS_BTN_LABEL = "->"
ROUGH_MINUS_BTN_LABEL = "<<-"
MINUS_BTN_LABEL = "<-"
CHANGE_PARAM_BTN_WID = 40
CHANGE_PARAM_BTN_HT = 20
PLUS_BTN_X = 240
MINUS_BTN_X = 80
ROUGH_PLUS_BTN_X = 300
ROUGH_MINUS_BTN_X = 20
FIRST_OBJ_Y = 20

#PARAM BOX
PARAM_BOX_X = 140
PARAM_BOX_WID = 80
PARAM_BOX_HT = 20



#PARAM LABEL
LABEL_X = 20
LABEL_WID = 100
LABEL_HT = 20
HT_MGN = 10

#SEND PARAM BUTTON
SEND_PARAM_BTN_X = 360
SEND_PARAM_BTN_Y = 20 + CONFIG_PARAM_NUM * (HT_PER_PARAM + HT_MGN + LABEL_HT) + 20
SEND_PARAM_BTN_WID = 120
SEND_PARAM_BTN_HT = 20
SEND_PARAM_BTN_NAME = "SEND PARAM"

#WINDOW
WIN_X_POS = 100
WIN_Y_POS = 100
WIN_WID = SEND_PARAM_BTN_X + SEND_PARAM_BTN_WID + 20
WIN_HT = SEND_PARAM_BTN_Y + SEND_PARAM_BTN_HT + 20
SYSTEM_NAME = "HARK PARAM RECONFIGURE"

def showCurrentParams():
	print "\n"
	for index in range(len(CONFIG_PARAMS)):
		print CONFIG_PARAMS_NAME[index] + ":" + str(CONFIG_PARAMS[index])
	print "\n"

#paramIndex:全パラメータリスト(HarkParamsDynReconfで定義された順番)の番号
#listIndex:GUIで表示するパラメータの番号
class ConfigParamInfo():
	def __init__(self,paramIndex,listIndex,paramName,defaultParam,minParam,maxParam,stride,roughStride):
		self.paramIndex,self.listIndex,self.paramName,self.defaultParam,self.minParam,self.maxParam,self.stride,self.roughStride = paramIndex,listIndex,paramName,defaultParam,minParam,maxParam,stride,roughStride

class MainWindow(QtGui.QWidget):
	def __init__(self):
		super(MainWindow, self).__init__()
		self.setParamInfo()
		self.initParamBtn()
		SendParamBtn(self)

	def changeValue(self, value):
		self.label.setText(str(value))
		msg = struct.pack("f"*len(CONFIG_PARAMS), *self.CONFIG_PARAMS)
		sock.send(msg)

	def getLabelYPos(self,index):
		return FIRST_OBJ_Y + index * (HT_PER_PARAM + LABEL_HT + HT_MGN)
	
	def getParamYPos(self,index):
		return FIRST_OBJ_Y + LABEL_HT + HT_MGN + index * (HT_PER_PARAM + LABEL_HT + HT_MGN)

	def setParamInfo(self):
# 		CONFIG_PARAM_INFO.append(ConfigParamInfo(1,"MIN_DEG",MIN_DEG,-180,180,1,None,None,None))
# 		CONFIG_PARAM_INFO.append(ConfigParamInfo(2,"MAX_DEG",MAX_DEG,-180,180,1,None,None,None))
# 		CONFIG_PARAM_INFO.append(ConfigParamInfo(3,"LOWER_BOUND_FREQUENCY",LOWER_BOUND_FREQUENCY,0,3000,100,None,None,None))
# 		CONFIG_PARAM_INFO.append(ConfigParamInfo(4,"UPPER_BOUND_FREQUENCY",UPPER_BOUND_FREQUENCY,0,3000,100,None,None,None))
		CONFIG_PARAM_INFO.append(ConfigParamInfo(5,0,"THRESH",THRESH,0,50,0.1,1.0))
		CONFIG_PARAM_INFO.append(ConfigParamInfo(0,1,"NUM_SOURCE",NUM_SOURCE,1,4,1,1))
		CONFIG_PARAM_INFO.append(ConfigParamInfo(6,2,"PAUSE_LENGTH",PAUSE_LENGTH,0,4000,10,100))
		CONFIG_PARAM_INFO.append(ConfigParamInfo(7,3,"MIN_SRC_INTERVAL",MIN_SRC_INTERVAL,0,500,10,50))
	
	def initParamBtn(self):
		for configParamInfo in CONFIG_PARAM_INFO:
			#PARAM LABEL
			label = QtGui.QLabel(configParamInfo.paramName,self)
			label.setGeometry(LABEL_X,self.getLabelYPos(configParamInfo.listIndex),LABEL_WID,LABEL_HT)
			
			#get param from args
			yPos = self.getParamYPos(configParamInfo.listIndex)
			roughStride = configParamInfo.roughStride
			stride = configParamInfo.stride
			
			#init param box
			paramBox = ParamBox(self,configParamInfo,yPos)
			PARAM_BOXES.append(paramBox)
			
			#init change param button
			ChangeParamButton(self,configParamInfo,paramBox, ROUGH_MINUS_BTN_X,yPos,-roughStride,ROUGH_MINUS_BTN_LABEL)
			ChangeParamButton(self,configParamInfo,paramBox, MINUS_BTN_X,yPos,-stride,MINUS_BTN_LABEL)
			ChangeParamButton(self,configParamInfo,paramBox, PLUS_BTN_X,yPos,stride,PLUS_BTN_LABEL)
			ChangeParamButton(self,configParamInfo,paramBox, ROUGH_PLUS_BTN_X,yPos,roughStride,ROUGH_PLUS_BTN_LABEL)

class SendParamBtn(QtGui.QPushButton):
	def __init__(self,parent):
		QtGui.QPushButton.__init__(self,parent)
		self.setGeometry(SEND_PARAM_BTN_X,SEND_PARAM_BTN_Y,SEND_PARAM_BTN_WID,SEND_PARAM_BTN_HT)
		self.setText(SEND_PARAM_BTN_NAME)
		self.clicked.connect(self.sendParam)
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.sock.connect((HOST, PORT))

	def sendParam(self):
		for paramBox in PARAM_BOXES:
			CONFIG_PARAMS[paramBox.paramIndex] = float(paramBox.text())
		msg = struct.pack("f"*len(CONFIG_PARAMS), *CONFIG_PARAMS)
		self.sock.send(msg)
		showCurrentParams()


class ParamBox(QtGui.QLineEdit):
	def __init__(self,parent,configParamInfo,yPos):
		QtGui.QLineEdit.__init__(self,parent)
		self.configParamInfo = configParamInfo
		self.paramIndex = configParamInfo.paramIndex
		self.setText(str(self.configParamInfo.defaultParam))
		self.setGeometry(PARAM_BOX_X,yPos,PARAM_BOX_WID,PARAM_BOX_HT)
		#self.show()


class ChangeParamButton(QtGui.QPushButton):
	def __init__(self,parent,configParamInfo,paramBox,xPos,yPos,stride,btnText):
		QtGui.QPushButton.__init__(self,parent)
		self.configParamInfo = configParamInfo
		self.stride = stride
		self.maxParam = configParamInfo.maxParam
		self.minParam = configParamInfo.minParam
		self.listIndex = configParamInfo.listIndex
		self.paramBox = paramBox
		#self.ifPlus = ifPlus
		self.xPos = xPos
		self.yPos = yPos
		
		self.setText(btnText)
		#self.setBtnText()
		self.setGeometry(self.xPos,self.yPos,CHANGE_PARAM_BTN_WID,CHANGE_PARAM_BTN_HT)
		self.clicked.connect(self.changeParam)
		self.show()

# 	def getXPos(self):
# 		if self.ifPlus is True:
# 			return PLUS_BTN_X
# 		else:
# 			return MINUS_BTN_X

	
# 	def setBtnText(self):
# 		if self.ifPlus:
# 			self.setText(PLUS_BTN_LABEL)
# 		else:
# 			self.setText(MINUS_BTN_LABEL)
	
	def changeParam(self):
		changedParam = float(str(self.paramBox.text())) + self.stride
		if changedParam <= self.maxParam and changedParam >= self.minParam:
			self.paramBox.setText(str(changedParam))

def initialize():

	app = QtGui.QApplication(sys.argv)
	window = MainWindow()
	window.setWindowTitle(SYSTEM_NAME)
	window.setGeometry(WIN_X_POS, WIN_Y_POS, WIN_WID, WIN_HT)
	window.show()
	return app,window


def finalize():
	sock.close()


if __name__ == '__main__':
	app,window = initialize()
	app.exec_()




