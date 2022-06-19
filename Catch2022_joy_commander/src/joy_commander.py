#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

class joy_commander:
    def __init__(self):
        rospy.init_node("joy_commander")
        rospy.loginfo("joy_commander : constructor activated")
        self.joy_input=rospy.Subscriber("/joy",Joy,self.joy_calback,queue_size=100)
        self.r=rospy.Rate(30)
        
        self.update()
        
    def joy_calback(self,msg):
        rospy.loginfo(msg)
        
    def update(self):
        while not rospy.is_shutdown():
            self.r.sleep()
                
        

if __name__=="__main__":
    try:
        rospy.loginfo("joy_commander : activated")
        inst = joy_commander()
    except:
        rospy.loginfo("something_wrong")
    finally:
        rospy.loginfo("process end")
        