#!/usr/bin/env python
# -*-coding: utf-8 -*-

import rospy
import roslib; roslib.load_manifest("telepabot")
from std_msgs.msg import String
import const
from hark_msgs.msg import HarkJuliusSrc
from hark_msgs.msg import HarkJuliusSrcVal

def talker():
	pub = rospy.Publisher(const.HARK_JULIUS_SOURCE_TOPIC_NAME, HarkJuliusSrc, queue_size=10)
	rospy.init_node('RecogResultPublisher', anonymous=True)
	r = rospy.Rate(1) # 10hz
	azimuth = 0
	srcId = 0
	
	while not rospy.is_shutdown():
		src = HarkJuliusSrc()
		src.id = srcId
		srcVal =HarkJuliusSrcVal()
		src.azimuth = azimuth % 50
		srcVal.word = "あいうえお"
		src.src = []
		src.src.append(srcVal)
		pub.publish(src)
		r.sleep()
		#azimuth+=1
		srcId+=1

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
