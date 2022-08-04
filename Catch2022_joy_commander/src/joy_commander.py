#!/usr/bin/env python3
#役割：ジョイスティック入力を指令値に換算

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import tf2_ros

class joy_commander:
    def __init__(self):
        rospy.loginfo("joy_commander : constructor activated")
        self.initializer()                
        self.update()
        
    def initializer(self):
        rospy.init_node("joy_commander")
        self.loop_rate = 30
        
        rospy.Subscriber("/manual_hand_cmd",  Float32MultiArray,self.manual_calback,queue_size=1)
        rospy.Subscriber("/manual_fine_tune", Float32MultiArray,self.fine_callback,queue_size=1)
        self.r = rospy.Rate(self.loop_rate)
        
        self.cmd_x:float = 0
        self.cmd_y:float = 0
        self.cmd_x_fine:float = 0
        self.cmd_y_fine:float = 0
        
        
    def fine_callback(self,msg):
        self.cmd_x_fine = 0.05*msg.data[0]
        self.cmd_y_fine = 0.05*msg.data[1]
    
    def manual_calback(self,msg):
        self.cmd_x = 0.5*msg.data[0]
        self.cmd_y = 0.5*msg.data[1]
    
    def update(self):
        while not rospy.is_shutdown():
            self.r.sleep()
                
if __name__=="__main__":
    rospy.loginfo("nya")
    try:
        rospy.loginfo("joy_commander : activated")
        inst = joy_commander()
    except:
        rospy.loginfo("something_wrong")
    finally:
        rospy.loginfo("process end")
        