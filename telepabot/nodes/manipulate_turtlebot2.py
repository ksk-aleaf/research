#!/usr/bin/python
# -*-coding: utf-8 -*-

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import const
import global_var
import rospy
import os
import time
import thetaimg
import format_loc_src_microcone
import math

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
		return const.STAY

	#上下方向と左右方向の入力値の差が閾値以上であれば、大きい入力の方を採用。斜め付近の入力は受け付けない。
	if abs(joyInput.frontBackInput) > abs(joyInput.leftRightInput) + const.JOY_PLAY_THRESHOLD:
		if joyInput.frontBackInput >= 0:
			return const.FRONT
		else:
			return const.BACK
	elif abs(joyInput.leftRightInput) > abs(joyInput.frontBackInput) + const.JOY_PLAY_THRESHOLD:
		if joyInput.leftRightInput >= 0:
			return const.LEFT
		else:
			return const.RIGHT
	else:
		return const.STAY

def autoRotateStarter():
	print "autoRotateStarter"
	
	#回転角度と回転方向を指定
	if global_var.listenSeparateSoundCount > 0:
		azimuth = (global_var.listenRangeList[const.MAIN_LISTEN_AREA].startAzimuth + global_var.listenRangeList[const.MAIN_LISTEN_AREA].endAzimuth) /2
	if azimuth > 0:
		global_var.robotMoveDirection = const.RIGHT
		global_var.robotPrevMoveDirection = const.RIGHT
	else:
		global_var.robotMoveDirection = const.LEFT
		global_var.robotPrevMoveDirection = const.LEFT
	global_var.autoRotateOdometry = math.fabs(const.ODOMETRY_PER_AZIMUTH * azimuth)
	print "autoRotateOdometry:"+str(global_var.autoRotateOdometry)
	
	#set auto rotate param
	global_var.autoRotateTimeout = abs(azimuth)*const.TO_PER_THETA - const.MAN_ROT_TO
	global_var.isAutoRotating = True
	global_var.autoRotateStartPeriod = time.time()
	format_loc_src_microcone.listenWholeSound()


def autoRotateFinisher():
	print "auto rotate finisher"
	
	global_var.isAutoRotating = False
	global_var.robotMoveDirection = const.JOY_STAY
	sendCommand(const.STOP_ROT_CMD)

	#回転分を調節
	if global_var.listenSeparateSoundCount > 0:
		format_loc_src_microcone.shiftListenRange(-getRotateAzimuth(global_var.odometryOrienZ))
		format_loc_src_microcone.listenSeparateSound()
	
	#format_loc_src_microcone.printListenRanges()#debug
	initRotateParam()

def manualRotateStarter():
	print "manualRotateStarter"
	#global_var.manualRotateDirection = global_var.robotMoveDirection
	global_var.robotPrevMoveDirection = global_var.robotMoveDirection


def manualRotateFinisher():
	print "manual rotate finisher"
	global_var.isManualRotating = False
	format_loc_src_microcone.shiftListenRange(-getRotateAzimuth(global_var.odometryOrienZ))
	initRotateParam()

def rotateFinisher():
	if global_var.isAutoRotating is True:
		autoRotateFinisher()
	elif global_var.isManualRotating is True:
		manualRotateFinisher()

def ifRotating():
	if global_var.isAutoRotating is True or global_var.isManualRotating is True:
		return True
	else:
		return False

#操作とマニュアル操作フラグが食い違っていたら合わせる
def checkManualRotatingFlag():
	if global_var.isManualRotating is False and (global_var.robotMoveDirection is const.RIGHT or global_var.robotMoveDirection is const.LEFT):
		global_var.isManualRotating = True
		manualRotateStarter()

def sendCommand(command):
	pub = rospy.Publisher(const.KOBUKI_VEL_NODE_STR, Twist,queue_size=10)
	pub.publish(command)
	#rospy.loginfo(command)


def sendRobotMoveDirection(direction):
	if direction is const.FRONT:
		sendCommand(const.FWD_CMD)
	elif direction is const.BACK:
		sendCommand(const.BACK_CMD)
	elif direction is const.LEFT:
		sendCommand(const.L_ROT_CMD)
	elif direction is const.RIGHT:
		sendCommand(const.R_ROT_CMD)
 	elif direction is const.STAY:
 		sendCommand(const.STOP_ROT_CMD)
	elif direction is not const.STAY:
		print "robot move direction error"

def joy_callback(joydata):	
	triggerButtonPushFlag = getButtonPushFlag(joydata.buttons[const.JOY_TRIGGER_BUTTON_INDEX])
	joyInput = JoyInput(joydata.axes[const.JOY_FRONT_BACK_INDEX],joydata.axes[const.JOY_LEFT_RIGHT_INDEX],triggerButtonPushFlag)
	
	#マニュアル入力終了時
	#if global_var.manualRotatingFlag is True and joyInput.frontBackInput == 0 and joyInput.leftRightInput == 0:
	#	sendCommand(const.STOP_ROT_CMD)
	
	#joy stick 入力受付
	if global_var.isAutoRotating is False:
		if joyInput.triggerButtonPushFlag is True:
			autoRotateStarter()
		else:#マニュアル入力受付
			global_var.robotMoveDirection = getDirection(joyInput)
			checkManualRotatingFlag()



def odometry_callback(odometry):
	
	#ODOMETRY_MAXを超えたら値をリセットしてカウンタを加算する
	if math.fabs(odometry.pose.pose.orientation.z) >= const.ODOMETRY_MAX and math.fabs(global_var.odometryOrienZ) < const.ODOMETRY_MAX:
		resetOdometry()
		global_var.odometryOverThresholdCount += 1

	#自動回転の終了判定
	if global_var.isAutoRotating is True:
		if getRotateOdometry(odometry.pose.pose.orientation.z) > global_var.autoRotateOdometry:
			autoRotateFinisher()

	#RESET_ODOMETRY_FRAME回だけ実行される間にodometryに変化がなければリセット
	if global_var.odometryOrienZ == odometry.pose.pose.orientation.z and odometry.pose.pose.orientation.z != 0:
		if global_var.resetOdometryCounter < const.RESET_ODOMETRY_FRAME:
			global_var.resetOdometryCounter += 1
		else:
			if global_var.isManualRotating is True:
				manualRotateFinisher()
			resetOdometry()
			initOdometryParam()
	else:#odometry値を取得
		global_var.odometryOrienZ = odometry.pose.pose.orientation.z
		global_var.resetOdometryCounter = 0

	if ifRotating() is True:
		sendRobotMoveDirection(global_var.robotMoveDirection)


def resetOdometry():
	print "----------------reset Odometry----------------"
	const.RESET_ODOMETRY_PUBLISHER.publish(const.RESET_ODOMETRY_COMMAND)


def initOdometryParam():
	global_var.odometryOverThresholdCount = 0
	global_var.resetOdometryCounter = 0


def initRotateParam():
	global_var.autoRotateOdometry = 0


#総回転odometryの絶対値を返す
def getRotateOdometry(currentOdometry):
	currentOdometryAbs = math.fabs(currentOdometry)

	#odometry値 = 現在のodometryの絶対値 + 閾値を超えてリセットされた回数 * 閾値
	odometry = currentOdometryAbs + global_var.odometryOverThresholdCount * const.ODOMETRY_MAX
	
	#まだリセットが適用されていないのでその分減算
	if currentOdometryAbs > const.ODOMETRY_MAX:
		odometry -= const.ODOMETRY_MAX
	
	return odometry

def getRotateAzimuth(currentOdometry):
	odometry = getRotateOdometry(currentOdometry)
	if global_var.robotPrevMoveDirection == const.LEFT:
		odometry *= -1	
	return odometry / const.ODOMETRY_PER_AZIMUTH

def subscriber():
	rospy.Subscriber(const.JOYSTICK_TOPIC_NAME, Joy, joy_callback, buff_size = 1)
	rospy.Subscriber(const.ODOMETRY_TOPIC_NAME, Odometry, odometry_callback, buff_size = 1)

def initializer():
	subscriber()
	resetOdometry()