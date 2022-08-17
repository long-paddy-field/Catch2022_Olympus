#!/usr/bin/env python3
#役割：ジョイコン入力の読み取り、各部への伝達（ボタン）

import queue
from numpy import empty
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class JOY_CONTROLER():   
    def __init__(self):
        self.initializer()
        self.update()
    
    def initializer(self):
        rospy.init_node("joy_controler")
        rospy.loginfo("init")
        #get_params
        self.loop_rate = 10
        
        # self.loop_rate = rospy.get_param("loop_rate")
        
        #handles
        self.pubLeftAxes = rospy.Publisher("manual_hand_cmd", Float32MultiArray, queue_size = 1)
        self.pubCross = rospy.Publisher("manual_fine_tune",Float32MultiArray,queue_size = 1)
        self.pubRotate = rospy.Publisher("manual_arm_rotate",Float32,queue_size = 1)
        self.pub_init_flag = rospy.Publisher("hard_init",Empty,queue_size = 1)
        self.pub_start_flag = rospy.Publisher("start_flag",Empty,queue_size = 1)
        self.pub_emergence_flag = rospy.Publisher("emergence_cmd",Empty,queue_size = 1)
        self.pub_isHandy = rospy.Publisher("isHandy",Bool,queue_size = 1)
        self.pub_grab_cmd = rospy.Publisher("grab_cmd",Empty,queue_size = 1)
        self.pub_release_cmd = rospy.Publisher("release_cmd",Empty,queue_size = 1)
        
        
        
        rospy.Subscriber("joy",Joy,self.joy_callback,queue_size = 1)
        self.r = rospy.Rate(self.loop_rate)
        
        #flags
        self.init_flag = True
        self.start_flag = False
        self.is_emergence= False
        self.isHandy = False
        self.isGrap = False
        
        #past_states
        self.past_isEnable = False
        
        #variables
        self.left_axes = list()
        self.right_axes = list()
        self.cross_axes = list()
        self.buttons = list()
        self.rot_ratio = 0.5 #rotationの最大速度との比率
        
    
    def joy_callback(self,msg):
        # rospy.loginfo("nya")
        self.left_axes = [-1*msg.axes[0],msg.axes[1]]
        self.right_axes = [-1*msg.axes[2],msg.axes[3]]
        self.cross_axes = [-1*msg.axes[4],msg.axes[5]]
        self.buttons = msg.buttons
    
        if msg.buttons[10]:# 左スティックで緊急停止（ソフト面）
            rospy.logwarn("emergency stop")
            self.is_emergence = not self.is_emergence
            self.pub_emergence_flag.publish()
        
        if not self.is_emergence:
            if msg.buttons[8]:# backでinit
                rospy.loginfo("init")
                self.pub_init_flag.publish()
            
            if msg.buttons[9]:# startでstart
                rospy.loginfo("start")
                self.pub_start_flag.publish()
                            
            
            if msg.buttons[11]:# 右スティックで手動と自動切り替え
                self.isHandy = not self.isHandy
                self.pub_isHandy.publish(self.isHandy)
                if self.isHandy:
                    rospy.loginfo("change auto -> hand")
                else :
                    rospy.loginfo("change handy -> auto")
            
                
    def update(self):
        rospy.loginfo("enter main routine")
        while not rospy.is_shutdown():
            rospy.loginfo(self.left_axes)
            rospy.loginfo(self.buttons)
            
            
            if not self.is_emergence:               
                f_msg = Float32MultiArray(data=self.left_axes)
                self.pubLeftAxes.publish(f_msg)

                f_msg = Float32MultiArray(data=self.cross_axes)
                self.pubCross.publish(f_msg)

                if not len(self.buttons) == 0:
                    if self.buttons[4] and not self.buttons[5]:
                        rospy.loginfo("Left")
                        f_msg = Float32(data = -1 * self.rot_ratio)
                        self.pubRotate.publish(f_msg)
                    elif self.buttons[5] and not self.buttons[4]:
                        rospy.loginfo("Right")
                        f_msg = Float32(data=self.rot_ratio)
                        self.pubRotate.publish(f_msg)
                    
            self.r.sleep()

if __name__ == '__main__':
    try:
        joy_con = JOY_CONTROLER()
    except:
        rospy.loginfo("joy_controler : something wrong")
    finally:
        rospy.loginfo("joy_controler : end process")