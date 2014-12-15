#! /usr/bin/env python
# -*-coding: utf-8 -*-

import csv
#import os.path
import const
import glob
import datetime

def getTimeStr():
	d = datetime.datetime.today()
	return d.strftime("%H:%M:%S")

filePointer = open(const.CSV_LOG_FILE_PATH + const.CSV_LOG_FILE_NAME_HEADER + getTimeStr() + const.CSV_LOG_FILE_EXT,"ab")
csvDataWriter = csv.writer(filePointer)

# def init():
# 	#for name in glob.glob(const.CSV_LOG_FILE_PATH + const.CSV_LOG_FILE_NAME_HEADER + "*"):
# 
# 	csvDataWriter = csv.writer(filePointer)




def writeLog(manipLabel,startAzimuth,endAzimuth):
	list = []
	list.append(getTimeStr())
	list.append(manipLabel)
	list.append(startAzimuth)
	list.append(endAzimuth)
	print "writeCsvList:"+str(list)
	csvDataWriter.writerow(list)

def close():
	filePointer.close()