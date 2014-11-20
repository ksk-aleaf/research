#!/usr/bin/python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import const
import global_var
import rospy
import os
import time
import thetaimg
import format_loc_src_microcone

class JoyInput():
	def __init__(self,frontBackInput,leftRightInput,triggerButtonPushFlag):
		self.frontBackInput,self.leftRightInput,self.triggerButtonPushFlag = frontBackInput,leftRightInput,triggerButtonPushFlag


# convert 0 or 1 to bool
def getButtonPushFlag(joyButtonData):
	if joyButtonData is const.JOY_BUTTON_PUSH:
		return True
	elif joyButtonData is const.JOY_BUTTON_RELEASE:
		return False
	else:
		print "error in judge button push"
		return False

# get direction for manipulate robot
def getDirection(joyInput):
	#入力値が遊びの範囲内であれば動かさない
	if abs(joyInput.frontBackInput) < const.JOY_PLAY_THRESHOLD and abs(joyInput.leftRightInput) < const.JOY_PLAY_THRESHOLD:
		return const.JOY_STAY

	#上下方向と左右方向の入力値の差が閾値以上であれば、大きい入力の方を採用。斜め付近の入力は受け付けない。
	if abs(joyInput.frontBackInput) > abs(joyInput.leftRightInput) + const.JOY_PLAY_THRESHOLD:
		if joyInput.frontBackInput >= 0:
			return const.JOY_FRONT
		else:
			return const.JOY_BACK
	elif abs(joyInput.leftRightInput) > abs(joyInput.frontBackInput) + const.JOY_PLAY_THRESHOLD:
		if joyInput.leftRightInput >= 0:
			return const.JOY_LEFT
		else:
			return const.JOY_RIGHT
	else:
		return const.JOY_STAY

def autoRotateStarter(joyInput):
	print "autoRotateStarter"
	
	#if joyInput.triggerButtonPushFlag is True and global_var.ifAutoRotate is not True:
	if global_var.listenSeparateSoundCount > 0:
		azimuth = (global_var.listenRangeList[0].startAzimuth + global_var.listenRangeList[0].endAzimuth) /2
	if azimuth > 0:
		global_var.robotMoveDirection = const.JOY_RIGHT
	else:
		global_var.robotMoveDirection = const.JOY_LEFT
	
	#set auto rotate param
	global_var.autoRotateTimeout = abs(azimuth)*const.TO_PER_THETA - const.MAN_ROT_TO
	global_var.ifAutoRotate = True
	global_var.autoRotateStartPeriod = time.time()
	format_loc_src_microcone.listenWholeSound()


def autoRotateFinisher():
	global_var.ifAutoRotate = False
	global_var.robotMoveDirection = const.JOY_STAY

	#回転分を調節
	format_loc_src_microcone.shiftListenRange(-(format_loc_src_microcone.getSeparateListenAngle()))
	format_loc_src_microcone.listenSeparateSound()


def manualRotateStarter():
	print "manualRotateStarter"
	global_var.manualRotateStartPeriod = time.time()
	global_var.manualRotateDirection = global_var.robotMoveDirection

def manualRotateFinisher():
	print "manualRotateFinisher"
# 	print "rotating time:" + str(time.time() - global_var.manualRotateStartPeriod)
# 	print "rotating azimuth:" + str((time.time() - global_var.manualRotateStartPeriod)/const.TO_PER_THETA)
	rotateAzimuth = (time.time() - global_var.manualRotateStartPeriod)/const.MAN_TO_PER_THETA# - 10
	if global_var.manualRotateDirection is const.JOY_RIGHT:
		rotateAzimuth = - rotateAzimuth
	format_loc_src_microcone.shiftListenRange(rotateAzimuth)

#操作とマニュアル操作フラグが食い違っていたら合わせる
def checkManualRotatingFlag():
# 	print "chekcManualRotating"
# 	print "manualRotatingFlag:"+str(global_var.manualRotatingFlag)
# 	print "robotMoveDirection:"+str(global_var.robotMoveDirection)
	#print "leftRightInput:"+str(joyInput.leftRightInput)
	
	if global_var.manualRotatingFlag is True and global_var.robotMoveDirection is const.JOY_STAY:
		global_var.manualRotatingFlag = False
		manualRotateFinisher()
	elif global_var.manualRotatingFlag is False and (global_var.robotMoveDirection is const.JOY_RIGHT or global_var.robotMoveDirection is const.JOY_LEFT):
		global_var.manualRotatingFlag = True
		manualRotateStarter()

#移動機構の操作
def manipulateOmni():
	if global_var.ifAutoRotate is False:#manual
		moveRobot(global_var.robotMoveDirection)
	else:#auto
		if time.time() - global_var.autoRotateStartPeriod > global_var.autoRotateTimeout:
			autoRotateFinisher()
		else:
			moveRobot(global_var.robotMoveDirection)


def sendCommand(command):
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist)
	pub.publish(command)
	#rospy.loginfo(command)

def moveRobot(direction):
	if direction is const.JOY_FRONT:
		sendCommand(const.FWD_CMD)
	elif direction is const.JOY_BACK:
		sendCommand(const.BACK_CMD)
	elif direction is const.JOY_LEFT:
		sendCommand(const.L_ROT_CMD)
	elif direction is const.JOY_RIGHT:
		sendCommand(const.R_ROT_CMD)
	elif direction is not const.JOY_STAY:
		print "robot move direction error"

def joy_callback(joydata):	
	triggerButtonPushFlag = getButtonPushFlag(joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])
	joyInput = JoyInput(joydata.axes[const.JOY_FRONT_BACK_INDEX],joydata.axes[const.JOY_LEFT_RIGHT_INDEX],triggerButtonPushFlag)
	
	if global_var.ifAutoRotate is False:
		if joyInput.triggerButtonPushFlag is True:
			autoRotateStarter(joyInput)
		else:
			global_var.robotMoveDirection = getDirection(joyInput)
			checkManualRotatingFlag()



def subscriber():
	rospy.Subscriber(const.JOYSTICK_TOPIC_NAME, Joy, joy_callback, buff_size = 1)

