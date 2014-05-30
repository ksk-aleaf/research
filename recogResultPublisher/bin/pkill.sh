#!/bin/sh
kill -9 `ps -al | grep recogResultPublisher | egrep -v grep | egrep -v rotatelogs2 | grep Main | awk '{print $2}'`
