#!/usr/bin/env python3
import rospy
import playsound
from std_msgs.msg import String


class Zunda():
    def __init__(self):
        rospy.Subscriber("zunda_call",String,self.zunda_callback(),queue_size=100)
        
        self.zunda_shout = "zunda"
        self.shout_flag  = False
        
        self.update()
        
    def zunda_callback(self,msg):
        self.shout_flag = True
        self.zunda_shout = msg.data
    
    def update(self):
        while not rospy.is_shutdown():
            if self.shout_flag:
                if self.zunda_shout == "connect":
                    pass

if __name__=='__main__':
    rospy.init_node("zunda_manager")
    arg = Zunda()