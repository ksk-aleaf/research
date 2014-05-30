#!/usr/bin/env python
import roslib; roslib.load_manifest('shunsuke')
import rospy
import threading
import time
from std_msgs.msg import String
from Tkinter import *

TmpText="Hello Keisuke"
win=None
SetText=None

def GetTkinterThread():
    import threading
    def TestTkinter():
        def greeting():
            global TmpText
            Label(win, text="Pushed").pack(side=TOP)
            print "Hello stdout world !"
        
        global TmpText
        global SetText
        win = Frame()
        win.pack()
        SetText=StringVar()
        SetText.set(TmpText)
        Label(win, textvariable=SetText).pack(side=TOP)
        # Button(win, text="Hello", command=greeting).pack(side=TOP)
        # Button(win, text="Quit", command=win.quit).pack(side=RIGHT)

        win.mainloop()
    #Run the mainloop in another thread
    t = threading.Thread(None, TestTkinter, 'Test Tkinter Thread')
    t.setDaemon(1)
    t.start()
    print 'Window Ready'
    return t 

def callback(data):
    global TmpText
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)
    TmpText = data.data
    SetText.set(TmpText)

def listener_py():
    rospy.init_node('listener_py', anonymous=True)
    rospy.Subscriber("chatter_py", String, callback)
    rospy.spin()

if __name__ == '__main__':
    t = GetTkinterThread() 
    listener_py()
