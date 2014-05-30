#! /usr/bin/env python
# -*-coding: utf-8 -*-

import roslib; roslib.load_manifest('ui_alt')
import rospy
import tf
import math
import sys
from ui_alt.msg import tf_uialt, src_tf_uialt
 
FRAMES = [
        'head',
        'neck',
        'torso'
        ]

LAST = rospy.Duration()

if __name__ == ('__main__'):
    
    rospy.init_node('tf_manager')

    pub_msg1 = tf_uialt()
    prev_pub_msg1 = tf_uialt()
    pub_msg2 = tf_uialt()
    prev_pub_msg2 = tf_uialt()
    pub_msg3 = tf_uialt()
    prev_pub_msg3 = tf_uialt()
    pub_msg4 = tf_uialt()
    prev_pub_msg4 = tf_uialt()
    listener = tf.TransformListener()

    duration1 = 10
    duration2 = 10
    duration3 = 10
    duration4 = 10
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        
        if listener.frameExists("/openni_depth_frame"):

            for frame in FRAMES:
                
                if listener.frameExists("/%s1" % (frame)):
                    trans, rot = listener.lookupTransform("/openni_depth_frame", "/%s1" % (frame), LAST)
                    flag = False
                    
                    for i in range(len(pub_msg1.src)):
                        
                        if pub_msg1.src[i].parts == '/%s1' % (frame):

                            
                            flag = True
                            pub_msg1.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            pub_msg1.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            pub_msg1.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))                   

                            if prev_pub_msg1.src[i].x == pub_msg1.src[i].x:
                                duration1 = duration1 - 1
                            else:
                                duration1 = 10
                            
                            prev_pub_msg1.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            prev_pub_msg1.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            prev_pub_msg1.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))                            

                    if flag == False:
                        append = src_tf_uialt()
                        append.parts = '/%s1' % (frame)
                        append.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        duration1 = 10
                        pub_msg1.src.append(append)
                        append1 = src_tf_uialt()
                        append1.parts = '/%s1' % (frame)
                        append1.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append1.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append1.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        prev_pub_msg1.src.append(append1)
                        
                if listener.frameExists("/%s2" % (frame)):
                    trans, rot = listener.lookupTransform("/openni_depth_frame", "/%s2" % (frame), LAST)
                    
                    flag = False
                    for i in range(len(pub_msg2.src)):
                        
                        if pub_msg2.src[i].parts == '/%s2' % (frame):

                            flag = True
                            pub_msg2.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            pub_msg2.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            pub_msg2.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))

                            if prev_pub_msg2.src[i].x == pub_msg2.src[i].x:
                                duration2 = duration2 - 1
                            else:
                                duration2 = 10
                            
                            prev_pub_msg2.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            prev_pub_msg2.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            prev_pub_msg2.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))                            


                    if flag == False:
                        append = src_tf_uialt()
                        append.parts = '/%s2' % (frame)
                        append.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        duration2 = 10
                        pub_msg2.src.append(append)
                        append1 = src_tf_uialt()
                        append1.parts = '/%s1' % (frame)
                        append1.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append1.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append1.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        prev_pub_msg2.src.append(append1)

                if listener.frameExists("/%s3" % (frame)):
                    trans, rot = listener.lookupTransform("/openni_depth_frame", "/%s3" % (frame), LAST)
                    
                    flag = False
                    for i in range(len(pub_msg3.src)):
                        
                        if pub_msg3.src[i].parts == '/%s3' % (frame):

                            flag = True
                            pub_msg3.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            pub_msg3.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            pub_msg3.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))

                            if prev_pub_msg3.src[i].x == pub_msg3.src[i].x:
                                duration3 = duration3 - 1
                            else:
                                duration3 = 10
                            
                            prev_pub_msg3.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            prev_pub_msg3.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            prev_pub_msg3.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))                            

                    if flag == False:
                        append = src_tf_uialt()
                        append.parts = '/%s3' % (frame)
                        append.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        duration3 = 10
                        pub_msg3.src.append(append) 
                        append1 = src_tf_uialt()
                        append1.parts = '/%s1' % (frame)
                        append1.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append1.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append1.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        prev_pub_msg3.src.append(append1)

                if listener.frameExists("/%s4" % (frame)):
                    trans, rot = listener.lookupTransform("/openni_depth_frame", "/%s4" % (frame), LAST)
                    
                    flag = False
                    for i in range(len(pub_msg4.src)):
                        
                        if pub_msg4.src[i].parts == '/%s4' % (frame):

                            flag = True
                            pub_msg4.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            pub_msg4.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            pub_msg4.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))

                            if prev_pub_msg4.src[i].x == pub_msg4.src[i].x:
                                duration4 = duration4 - 1
                            else:
                                duration4 = 10
                            
                            prev_pub_msg4.src[i].x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                            prev_pub_msg4.src[i].y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                            prev_pub_msg4.src[i].theta = -math.degrees(math.atan(trans[0] / trans[2]))                            

                    if flag == False:
                        append = src_tf_uialt()
                        append.parts = '/%s4' % (frame)
                        append.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        duration4 = 10
                        pub_msg4.src.append(append) 
                        append1 = src_tf_uialt()
                        append1.parts = '/%s1' % (frame)
                        append1.x     = 640 - 320 * (1 - (math.tan(trans[0] / trans[2]) / math.tan(math.radians(31.5))))
                        append1.y     = 240 * (1 - (math.tan(trans[1] / trans[2]) / math.tan(math.radians(22))))
                        append1.theta = -math.degrees(math.atan(trans[0] / trans[2]))
                        prev_pub_msg4.src.append(append1)

            pub1 = rospy.Publisher('tf_processed1', tf_uialt)
            pub2 = rospy.Publisher('tf_processed2', tf_uialt)
            pub3 = rospy.Publisher('tf_processed3', tf_uialt)
            pub4 = rospy.Publisher('tf_processed4', tf_uialt)
            if duration1 > 0:
                pub1.publish(pub_msg1)
            elif duration1 <= 0:
                pub_msg_null1 = tf_uialt()
                pub1.publish(pub_msg_null1)
            if duration2 > 0:
                pub2.publish(pub_msg2)
            elif duration2 <= 0:
                pub_msg_null2 = tf_uialt()
                pub2.publish(pub_msg_null2)
            if duration3 > 0: 
                pub3.publish(pub_msg3)
            elif duration3 <= 0:
                pub_msg_null3 = tf_uialt()
                pub3.publish(pub_msg_null3)
            if duration4 > 0:
                pub4.publish(pub_msg4)
            elif duration4 <= 0:
                pub_msg_null4 = tf_uialt()
                pub4.publish(pub_msg_null4)
            rate.sleep()


         

    
