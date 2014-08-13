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
	#if joyInput.triggerButtonPushFlag is True and global_var.ifAutoRotate is not True:
	azimuth = (global_var.listenRangeStartAngle + global_var.listenRangeEndAngle) /2
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
	#回転に合わせて視聴範囲を移動
	azimuth = (global_var.listenRangeStartAngle + global_var.listenRangeEndAngle) /2
	global_var.listenRangeStartAngle = global_var.listenRangeStartAngle - azimuth
	global_var.listenRangeEndAngle = global_var.listenRangeEndAngle - azimuth
	format_loc_src_microcone.setListenAxis(global_var.listenRangeStartAngle,global_var.listenRangeEndAngle)
	format_loc_src_microcone.listenSeparateSound()
# 	global_var.listenRangeStartAngle = -10
# 	global_var.listenRangeStartX = thetaimg.getXAxisFromAzimuth(global_var.listenRangeStartAngle)
# 	global_var.listenRangeEndAngle = 10
# 	global_var.listenRangeEndX = thetaimg.getXAxisFromAzimuth(global_var.listenRangeEndAngle)	

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


def subscriber():
	rospy.Subscriber(const.JOYSTICK_TOPIC_NAME, Joy, joy_callback, buff_size = 1)

