#! /usr/bin/env python
# -*- coding:utf-8 -*-

'''
KINECT + ROS + OpenNI sample
https://gist.github.com/2414166
'''

import roslib; roslib.load_manifest('ui_alt')
import rospy
import tf

BASE_FRAME = '/openni_depth_frame'
FRAMES = [
        'head',
        'neck',
        'torso',
        'left_shoulder',
        'left_elbow',
        'left_hand',
        'left_hip',
        'left_knee',
        'left_foot',
        'right_shoulder',
        'right_elbow',
        'right_hand',
        'right_hip',
        'right_knee',
        'right_foot',
        ]
LAST = rospy.Duration()


class Kinect:

    def __init__(self, name='kinect_listener', user=1):
        rospy.init_node(name, anonymous=True)
        self.listener = tf.TransformListener()
        self.user = user

    
    def get_posture(self):
        """Returns a list of frames constituted by a translation matrix
        and a rotation matrix.

        Raises IndexError when a frame can't be found (which happens if
        the requested user is not calibrated).
        """
        frames = []
        while not self.listener.frameExists(BASE_FRAME):
            print 'no'
        #print "yes"
        for frame in FRAMES:
            trans, rot = self.listener.lookupTransform(BASE_FRAME,"/%s_%d" % (frame, self.user), LAST)
            #trans, rot = self.listener.lookupTransform(BASE_FRAME, "/%s" %(frame), LAST)
            frames.append((trans, rot))
        print frames[14][0][1]       
        return frames
'''except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException):
            print 'Error'
'''
if __name__ == '__main__':
    
    
    kin = Kinect()
    for i in range (1000):
        kin.get_posture()
