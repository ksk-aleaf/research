#-*- coding: utf-8 -*-
import rospy
import global_var
import const
import time
import thetaimg
from hark_msgs.msg import HarkJuliusSrc  # @UnresolvedImport

from PyQt4 import QtCore
from PyQt4 import QtGui
from PyQt4.QtCore import *
from PyQt4.QtGui import *

#recognistion result data class
class RecogWord():
	def __init__(self,sourceId,text,azimuth,boundBox,startX,endX,timestamp):
		self.sourceId,self.text,self.azimuth,self.boundBox,self.startX,self.endX,self.timestamp = sourceId,text,azimuth,boundBox,startX,endX,timestamp
#	def __init__(self,sourceId,text,azimuth,vertPosIndex,boundBox,startX,endX,timestamp):
#		self.sourceId,self.text,self.azimuth,self.vertPosIndex,self.boundBox,self.startX,self.endX,self.timestamp = sourceId,text,azimuth,vertPosIndex,boundBox,startX,endX,timestamp

#param String
#return QSize
def getWordDrawSize(word):
	return QSize(getWordDrawWidth(word),const.HEIGHT_PER_CHAR)

#~ def getWordDrawHeight(index):
	#~ yaxis = const.RECOG_DRAW_BTM_YAXIS - index
	#~ if yaxis < 0:
		#~ yaxis = 0
	#~ return  yaxis

def getWordDrawWidth(word):
	return len(word)*const.WIDTH_PER_CHAR


def getWordDrawPoint(azimuth,width):
	xaxis = thetaimg.getXAxisFromAzimuth(azimuth) - (width / 2 )

	if xaxis < 0:
		xaxis = 0
	elif xaxis > const.WIN_WID - width - 1:
		xaxis = const.WIN_WID - width - 1

	yaxis = const.RECOG_DRAW_BTM_YAXIS - const.RECOG_WORD_HEIGHT_RANGE - const.VERT_WORD_MGN
	return QPoint(xaxis,yaxis)


#位置がかぶっているワードがあれば古い方を上にする。一番上であれば消去する。
def adjustWordsPosition():
	for vertPosIndex in range(0,const.VERT_MAX_WORD_NUM):
		while 1:#ワードを削除した場合に最初から見直すためのループ
			breakFlag = True
			for wordIndex in range(0,len(global_var.recogWordList[vertPosIndex]) -1 ):
				word = global_var.recogWordList[vertPosIndex][wordIndex]
				nextWord = global_var.recogWordList[vertPosIndex][wordIndex+1]
				
				#ワードの位置がかぶっているかを判定
				print "word.startX:"+str(word.startX)
				print "nextWord.startX:"+str(nextWord.startX)
				if word.startX < nextWord.endX and word.endX > nextWord.startX:
					#print "inif1"
					adjustWord = None
					breakFlag = False
					
					#どちらが古いかでずらすワードを変える
					if word.timestamp > nextWord.timestamp:
						adjustWord = nextWord
					else:
						adjustWord = word
					
					#ワードをリストから削除して上にずらす(垂直位置が一番上の場合は消しっぱなし)
					global_var.recogWordList[vertPosIndex].remove(adjustWord)
					if vertPosIndex < const.VERT_MAX_WORD_NUM -1:
						#print "inif2"
						topLeft = QPoint(adjustWord.boundBox.topLeft().x(),adjustWord.boundBox.topLeft().y() - const.RECOG_WORD_HEIGHT_RANGE - const.VERT_WORD_MGN)
						bound = QRect(topLeft,adjustWord.boundBox.size())
						adjustWord.boundBox = bound
						#adjustWord.boundBox.bottomLeft().setY(adjustWord.boundBox.bottomLeft().y() - const.RECOG_WORD_HEIGHT_RANGE - const.VERT_WORD_MGN)
						#print "adjustWord.Y:"+str(adjustWord.boundBox.bottomLeft().y())
						appendWord(vertPosIndex+1,adjustWord)
					#水平位置のワードリストを最初から見直す為にforループを抜ける
					break
			#ワードの削除が起こらなければ垂直位置を上にずらす為にループ(while 1)を抜ける
			if breakFlag:
				break


#def sortWordsByXaxis():
#	global_var.recogWordList
#	sorted(student_objects, key=attrgetter('age'), reverse=True)

def getBoundBox(azimuth,word):
	size = getWordDrawSize(word)
	point = getWordDrawPoint(azimuth,size.width())
	return QRect(point,size)


def initRecogData():
	for index in range(0,const.VERT_MAX_WORD_NUM):
		global_var.recogWordList.append([])


def appendWord(vertPosIndex,appendWord):
	index = 0
	for word in global_var.recogWordList[vertPosIndex]:
		if word.startX > appendWord.startX:
			break
		index+=1
	global_var.recogWordList[vertPosIndex].insert(index,appendWord)

def recogword_callback(recogData):
	sourceId = recogData.id
	text = recogData.src[0].word
	azimuth = recogData.azimuth
	bound = getBoundBox(azimuth,text)
	startX = bound.bottomLeft().x()
	endX = startX + bound.width()
	timestamp = time.time()
	vertPosIndex = 0
	print "word:"+text
	appendWord(vertPosIndex,RecogWord(sourceId,text,azimuth,bound,startX,endX,timestamp))
	#appendWord(vertPosIndex = 0,RecogWord(sourceId,text,azimuth,vertPosIndex = 0,bound,startX,endX,timestamp))
	#global_var.recogWordList[0].append(RecogWord(sourceId,text,azimuth,bound,startX,endX,timestamp))
	#adjustWordsPosition()
	#print "recog_callback:" + text


def subscriber():
	rospy.Subscriber(const.HARK_JULIUS_SOURCE_TOPIC_NAME, HarkJuliusSrc, recogword_callback, buff_size = 1)
