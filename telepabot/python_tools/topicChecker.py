#!/usr/bin/python
# -*-coding: utf-8 -*-

import commands
import threading
import rospy
import time
import signal
from std_msgs.msg import Bool

#定数
NODE_ALIVE_CHECK_TIME_INTERVAL = 1.0
STATUS_PUBLISH_TIME_INTERVAL = 1.0
ROSNODE_PING_COMMAND = "rosnode ping -c 1 "
PING_FAILED_STR = "cannot ping"
#LOCALIZATION_TOPIC_NAME = "/HARK_MASTER_NODE2"
LOCALIZATION_TOPIC_NAME = "/HARK_MASTER_NODE2"
#SEPARATION_TOPIC_NAME = "/HARK_SEPARATE_PLAY_NODE"
SEPARATION_TOPIC_NAME = "/SEPARATE"
SYSTEM_STATUS_TOPIC_NAME = "SYSTEM_STATUS"
LOCALIZATION_FLAG_INDEX = 0
SEPARATION_FLAG_INDEX = 1

#グローバル変数
aliveFlags = [True,True]
finalizeFlag = False

#システムの正常・異常(所定のノードが動いているか)をBoolでパブリッシュ
class SystemStatusPublisher(threading.Thread):
	def __init__(self):
		super(SystemStatusPublisher, self).__init__()
		self.setDaemon(True)
		rospy.init_node(SYSTEM_STATUS_TOPIC_NAME, anonymous=True)
		self.publisher = rospy.Publisher(SYSTEM_STATUS_TOPIC_NAME, Bool, queue_size=10)

	def isSystemCorrect(self):
		global aliveFlags
		systemCorrectFlag = True

		for flag in aliveFlags:
			if flag is False:
				systemCorrectFlag = False
		
		return systemCorrectFlag
	
	def run(self):
		while True:
			self.publisher.publish(self.isSystemCorrect())
			time.sleep(STATUS_PUBLISH_TIME_INTERVAL)
	
#各ノードにpingを送って生存確認する
class NodeCheckThread(threading.Thread):

	def __init__(self, timeInterval,nodeName,flagIndex):
		super(NodeCheckThread, self).__init__()
		self.setDaemon(True)
		self.timeInterval = timeInterval
		self.nodeName = nodeName
		self.flagIndex = flagIndex

	def run(self):
		global aliveFlags
		print self.nodeName + ":check start"
		while True:
			time.sleep(self.timeInterval)
			result = commands.getoutput(ROSNODE_PING_COMMAND + self.nodeName)
			if result[0:len(PING_FAILED_STR)] != PING_FAILED_STR:
				aliveFlags[self.flagIndex] = True
			else:
				print self.nodeName + ":ping has no response"
				aliveFlags[self.flagIndex] = False

def finalize(num,frame):
	global finalizeFlag
	finalizeFlag = True

if __name__ == '__main__':
	global finalizeFlag
	signal.signal(signal.SIGINT,finalize)#シグナル(Ctrl-C 等)が入力されたら指定関数を実行
	systemStatusPublisher = SystemStatusPublisher()
	localizationNodeChecker = NodeCheckThread(NODE_ALIVE_CHECK_TIME_INTERVAL,LOCALIZATION_TOPIC_NAME,LOCALIZATION_FLAG_INDEX)
	separateNodeChecker = NodeCheckThread(NODE_ALIVE_CHECK_TIME_INTERVAL,SEPARATION_TOPIC_NAME,SEPARATION_FLAG_INDEX)

	#スレッド開始
	localizationNodeChecker.start()
	separateNodeChecker.start()
	systemStatusPublisher.start()

	#Ctrl-C が入力されたら終了
	while True:
		if finalizeFlag:
			break
