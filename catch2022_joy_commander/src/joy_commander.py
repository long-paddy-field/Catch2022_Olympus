#!/usr/bin/env python3
#役割：ジョイスティック入力を指令値に換算

# モーターがmove_cmd float32multiarray
# サーボ   servo_angle float32
# ステッパ　stepper_state int8
# ポンプ    pmp_state bool

import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class JOY_COMMANDER():
    def __init__(self):
        rospy.loginfo("joy_commander : start_process")
        self.initializer()
        self.update()
    
    def initializer(self):
        self.r = rospy.Rate(10)
        
        self.pub_move_cmd        = rospy.Publisher("move_cmd",Float32MultiArray,queue_size = 1)
        self.pub_servo_angle     = rospy.Publisher("servo_angle",Float32,queue_size = 1)
        self.pub_stepper_state   = rospy.Publisher("stepper_state",Int8,queue_size = 1)
        self.pub_pmp_state       = rospy.Publisher("pmp_state",Bool,queue_size = 1)
        
        rospy.Publisher("left_axes",Float32MultiArray,self.left_axes_callback,queue_size=1)
                
    
    def update(self):
        while not rospy.is_shutdown():
            self.r.sleep()
            
        

if __name__=='__main__':
    rospy.init_node('joy_commander')
    joy_commander = JOY_COMMANDER()
    rospy.loginfo("joy_commander : node is activated")
    rospy.loginfo("joy_commander : end_process")