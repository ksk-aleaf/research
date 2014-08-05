#!/usr/bin/python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import const
import global_var
import rospy

class JoyInput():
	def __init__(self,frontBackInput,leftRightInput,triggerButtonPushFlag):
		self.frontBackInput,self.leftRightInput,self.triggerButtonPushFlag = frontBackInput,leftRightInput,triggerButtonPushFlag

# convert 0 or 1 to bool
def getButtonPushFlag(joyButtonData):
	if joyButtonData is const.JOY_BUTTON_PUSH:
		return true
	elif joyButtonData is const.JOY_BUTTON_RELEASE:
		return false
	else:
		print "error in judge button push"
		return false

# get direction for manipulate robot
def getDirection(joyInput):
	if abs(joyInput.frontBackInput) > abs(joyInput.leftRightInput):
		if joyInput.frontBackInput >= 0:
			return const.JOY_FRONT
		else:
			return const.JOY_BACK
	else:
		if joyInput.leftRightInput >= 0:
			return const.JOY_LEFT
		else:
			return const.JOY_RIGHT

def sendCommand(command):
	#os.system(const.SET_TO_STR + str(timeout))
	#pub = rospy.Publisher('/cmd_vel', Twist)
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist)
	pub.publish(command)
	rospy.loginfo(command)

def moveRobot(direction):
	if direction is const.JOY_FRONT:
		sendCommand(const.FWD_CMD)
	elif direction is const.JOY_BACK:
		sendCommand(const.BACK_CMD)
	elif direction is const.JOY_LEFT:
		sendCommand(const.L_ROT_CMD)
	elif direction is const.JOY_RIGHT:
		sendCommand(const.R_ROT_CMD)
	else:
		print "robot move direction error"

def joy_callback(joydata):
	print "joy_callback"
	joyInput = JoyInput(joydata.axes[const.JOY_FRONT_BACK_INDEX],joydata.axes[const.JOY_LEFT_RIGHT_INDEX],joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])
	moveRobot(getDirection(joyInput))
# 	global_var.joyInput.frontBackInput = joydata.axes[const.JOY_FRONT_BACK_INDEX]
# 	global_var.joyInput.leftRightInput = joydata.axes[const.JOY_LEFT_RIGHT_INDEX]
# 	global_var.joyInput.triggerButtonPushFlag = getButtonPushFlag(joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])

def subscriber():
	rospy.Subscriber(const.JOYSTICK_TOPIC_NAME, Joy, joy_callback, buff_size = 1)

#def initialize():
