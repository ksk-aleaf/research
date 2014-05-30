#! /usr/bin/env python
# coding = utf8

import roslib; roslib.load_manifest('shunsuke')
import rospy
from hearbo_utils.msg import RobotStateAnglesVelocities
import Tkinter as tk

DIRECTION = ['LEFT','CENTER','RIGHT']

class PublishRobotCommands: 
    def __init__(self, direction):
        self.direction = direction

    def __call__(self, event = None):
        try:
            pub = rospy.Publisher('position_commands',RobotStateAnglesVelocities)
            rospy.init_node('UIcontrol')
            msg = RobotStateAnglesVelocities()

            # Commands decided by button action. 

            if self.direction == 'LEFT':

                msg.angles.left_arm[0].valid = True
                msg.angles.left_arm[0].value = -50.0/180.0*3.1415
                msg.velocities.left_arm[0].valid = True
                msg.velocities.left_arm[0].value = 500.0/1000.0

                msg.angles.neck[2].valid = True
                msg.angles.neck[2].value = 30/180.0*3.1415
                msg.velocities.neck[2].valid = True
                msg.velocities.neck[2].value = 500.0/1000.0

            if self.direction == 'CENTER':
                
                msg.angles.neck[1].valid = True
                msg.angles.neck[1].value = -25.0/180.0*3.1415
                msg.velocities.neck[1].valid = True
                msg.velocities.neck[1].value = 500.0/1000.0
                
            # if self.direction == 'RIGHT':

            rospy.loginfo(msg)
            pub.publish(msg)
            print('message to the robot sent!')
            rospy.sleep(1.0)

        except rospy.ROSInterruptException: pass

class Frame(tk.Frame):
    def __init__(self, master = None):
        tk.Frame.__init__(self, master)
        self.master.title('Control UI')

        fbutton = tk.Frame(self)
        fbutton.pack()

        for d in DIRECTION:
            b = tk.Button(fbutton, text = d, command = PublishRobotCommands(d))
            b.pack(side = tk.LEFT, padx = 3)

if __name__ == '__main__':
    
    f = Frame()
    f.pack()
    f.mainloop()
