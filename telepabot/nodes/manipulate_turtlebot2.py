#!/usr/bin/python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import const
import global_var
import rospy
import os
import time

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

def autoRotate(joyInput):
	print "trigbtnpushflg:"+str(joyInput.triggerButtonPushFlag)
	if joyInput.triggerButtonPushFlag is True and global_var.ifAutoRotate is not True:
		azimuth = (global_var.listenRangeStartAngle + global_var.listenRangeEndAngle) /2
		if azimuth > 0:
			global_var.robotMoveDirection = const.JOY_RIGHT
			command = const.R_ROT_CMD
		else:
			global_var.robotMoveDirection = const.JOY_LEFT
			command = const.L_ROT_CMD
		
		global_var.autoRotateTimeout = abs(azimuth)*const.TO_PER_THETA - const.MAN_ROT_TO
		#os.system(const.SET_TO_STR + str(global_var.autoRotateTimeout))
		#print "timeout:"+str(global_var.autoRotateTimeout)
		#sendCommand(command)
		global_var.autoRotateStartPeriod = time.time()
		

def sendCommand(command):
	#os.system(const.SET_TO_STR + str(timeout))
	#pub = rospy.Publisher('/cmd_vel', Twist)
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist)
	pub.publish(command)
	rospy.loginfo(command)

def moveRobot(direction):
	#os.system(const.SET_TO_STR + str(const.MAN_ROT_TO))
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
	#print "joy_callback"
	triggerButtonPushFlag = getButtonPushFlag(joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])
	joyInput = JoyInput(joydata.axes[const.JOY_FRONT_BACK_INDEX],joydata.axes[const.JOY_LEFT_RIGHT_INDEX],triggerButtonPushFlag)
	global_var.robotMoveDirection = getDirection(joyInput)
	autoRotate(joyInput)
	#print "input:"+str(joyInput)
	#print "direction"+str(global_var.robotMoveDirection)
	#moveRobot(getDirection(joyInput))
# 	global_var.joyInput.frontBackInput = joydata.axes[const.JOY_FRONT_BACK_INDEX]
# 	global_var.joyInput.leftRightInput = joydata.axes[const.JOY_LEFT_RIGHT_INDEX]
# 	global_var.joyInput.triggerButtonPushFlag = getButtonPushFlag(joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])

def subscriber():
	rospy.Subscriber(const.JOYSTICK_TOPIC_NAME, Joy, joy_callback, buff_size = 1)

#def initialize():
