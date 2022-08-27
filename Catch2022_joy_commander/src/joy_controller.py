#!/usr/bin/env python3
#役割：ジョイコン入力の読み取り、各部への伝達（ボタン）

import queue
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class JOY_CONTROLLER():
    def __init__(self):
        rospy.loginfo("joy_controller : start process")
        self.initializer()
        self.update()
        
    def initializer(self):
        self.r = rospy.Rate(10)
        
        self.pub_left_axes      = rospy.Publisher("left_axes",Float32MultiArray,queue_size = 1)
        self.pub_right_axes     = rospy.Publisher("right_axes",Float32MultiArray,queue_size = 1)
        self.pub_fine_axes      = rospy.Publisher("fine_axes",Float32MultiArray,queue_size = 1)
        self.pub_field_color    = rospy.Publisher("field_color",String,queue_size = 1) # フィールドの色を設定
        self.pub_grab_cmd       = rospy.Publisher("grab_cmd",Empty,queue_size = 1)     # 掴む
        self.pub_release_cmd    = rospy.Publisher("release_cmd",Empty,queue_size = 1)  # 離す
        self.pub_servo_cmd      = rospy.Publisher("servo_cmd",Int8,queue_size = 1)     # サーボ 1 が CCW 0
        self.pub_step_cmd       = rospy.Publisher("step_cmd",Int8,queue_size = 1)      # ステッパ 1 が 上昇
        self.pub_emergence_cmd  = rospy.Publisher("emergence_cmd",Int8,queue_size = 1)  # 緊急停止
        self.pub_start_flag     = rospy.Publisher("start_flag",Empty,queue_size = 1)   # スタート
        self.pub_is_Handy       = rospy.Publisher("is_handy",Bool,queue_size = 1)      # 手動自動切り替え
        
        rospy.Subscriber("joy", Joy,self.joy_callback,queue_size = 1)
        
        self.left_axes      = list()
        self.right_axes     = list()
        self.cross_axes     = list()
        self.buttons        = list()
        self.field_color    = String()
        self.servo_cmd      = Int8()
        self.step_cmd       = Int8()
        self.emergence_cmd  = Int8()
        self.is_Handy       = Bool()
        
        self.field_color.data   = 'Red'
        self.servo_cmd.data     = 0
        self.step_cmd.data      = 0
        self.emergence_cmd      = 0xff
        self.is_Handy           = False
        self.emergence          = False
        rospy.loginfo("joy_controller : end setting")
        
    def joy_callback(self,msg):
        self.left_axes  = [-1*msg.axes[0],msg.axes[1]]
        self.right_axes = [-1*msg.axes[2],msg.axes[3]]
        self.cross_axes = [-1*msg.axes[4],msg.axes[5]]
        self.buttons = msg.buttons
        
    def update(self):
        rospy.loginfo("joy_controller : main routine")
        while not rospy.is_shutdown():
            if not self.emergence and not len(self.buttons) == 0:               
                f_msg = Float32MultiArray(data=self.left_axes)
                self.pub_left_axes.publish(f_msg)
                
                f_msg = Float32MultiArray(data=self.right_axes)
                self.pub_right_axes.publish(f_msg)
                
                f_msg = Float32MultiArray(data=self.cross_axes)
                self.pub_fine_axes.publish(f_msg)
                
                
                if self.buttons[0] and not self.buttons[2]: #青サイドのとき押す
                    self.field_color.data = 'Blue'
                    self.pub_field_color.publish(self.field_color)
                
                if self.buttons[1] and not self.buttons[3]: #掴む
                    self.pub_grab_cmd.publish()
                    
                if self.buttons[2] and not self.buttons[0]: #赤サイドのとき押す
                    self.field_color.data = 'Red'
                    self.pub_field_color.publish(self.field_color)
                    
                if self.buttons[3] and not self.buttons[1]: #離す
                    self.pub_release_cmd.publish()
                    
                if self.buttons[4] and not self.buttons[5]: #サーボCCW
                    self.servo_cmd.data = 1
                    self.pub_servo_cmd.publish(self.servo_cmd)
                    
                if self.buttons[5] and not self.buttons[4]: #サーボCW
                    self.servo_cmd.data = -1
                    self.pub_servo_cmd.publish(self.servo_cmd)
                    
                if self.buttons[6] and not self.buttons[7]: #ステップ上昇
                    self.step_cmd.data = 1
                    self.pub_step_cmd.publish(self.step_cmd)
                    
                if self.buttons[7] and not self.buttons[6]: #ステップ下降
                    self.step_cmd.data = -1
                    self.pub_step_cmd.publish(self.step_cmd)
                                        
                if self.buttons[9]:                         #動作開始
                    self.pub_start_flag.publish()
                
                if self.buttons[11]:                        #自動手動切り替え
                    self.is_Handy = not self.is_Handy               
                    if self.is_Handy:
                        rospy.loginfo("change auto -> hand")
                    else :
                        rospy.loginfo("change handy -> auto")
                self.pub_is_Handy.publish(self.is_Handy)
                        
            if not len(self.buttons) == 0 and self.buttons[8]:                         #緊急停止
                self.pub_emergence_cmd.publish(self.emergence_cmd)
                self.emergence = not self.emergence            
            
            rospy.loginfo(self.left_axes)
            rospy.loginfo(self.cross_axes)
            rospy.loginfo(self.buttons)
            self.r.sleep()
        

if __name__ == '__main__':
    rospy.init_node('joy_controller')
    joy_con = JOY_CONTROLLER()
    rospy.loginfo("joy_controller : end process")
    
    

# import queue
# from numpy import empty
# import rospy
# from sensor_msgs.msg import Joy
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import Empty
# from std_msgs.msg import Bool
# from std_msgs.msg import Float32

# class JOY_CONTROLLER():   
#     def __init__(self):
#         self.initializer()
#         self.update()
    
#     def initializer(self):
#         rospy.init_node("joy_controller")
#         rospy.loginfo("init")
#         #get_params
#         self.loop_rate = 10
        
#         # self.loop_rate = rospy.get_param("loop_rate")
        
#         #handles
#         self.pubLeftAxes = rospy.Publisher("manual_hand_cmd", Float32MultiArray, queue_size = 1)
#         self.pubCross = rospy.Publisher("manual_fine_tune",Float32MultiArray,queue_size = 1)
#         self.pubRotate = rospy.Publisher("manual_arm_rotate",Float32,queue_size = 1)
#         self.pub_init_flag = rospy.Publisher("hard_init",Empty,queue_size = 1)
#         self.pub_start_flag = rospy.Publisher("start_flag",Empty,queue_size = 1)
#         self.pub_emergence_flag = rospy.Publisher("emergence_cmd",Empty,queue_size = 1)
#         self.pub_isHandy = rospy.Publisher("isHandy",Bool,queue_size = 1)
#         self.pub_grab_cmd = rospy.Publisher("grab_cmd",Empty,queue_size = 1)
#         self.pub_release_cmd = rospy.Publisher("release_cmd",Empty,queue_size = 1)
        
        
        
#         rospy.Subscriber("joy",Joy,self.joy_callback,queue_size = 1)
#         self.r = rospy.Rate(self.loop_rate)
        
#         #flags
#         self.init_flag = True
#         self.start_flag = False
#         self.is_emergence= False
#         self.isHandy = False
#         self.isGrap = False
        
#         #past_states
#         self.past_isEnable = False
        
#         #variables
#         self.left_axes = list()
#         self.right_axes = list()
#         self.cross_axes = list()
#         self.buttons = list()
#         self.rot_ratio = 0.5 #rotationの最大速度との比率
        
    
#     def joy_callback(self,msg):
#         # rospy.loginfo("nya")
#         self.left_axes = [-1*msg.axes[0],msg.axes[1]]
#         self.right_axes = [-1*msg.axes[2],msg.axes[3]]
#         self.cross_axes = [-1*msg.axes[4],msg.axes[5]]
#         self.buttons = msg.buttons
    
#         if msg.buttons[10]:# 左スティックで緊急停止（ソフト面）
#             rospy.logwarn("emergency stop")
#             self.is_emergence = not self.is_emergence
#             self.pub_emergence_flag.publish()
        
#         if not self.is_emergence:
#             if msg.buttons[8]:# backでinit
#                 rospy.loginfo("init")
#                 self.pub_init_flag.publish()
            
#             if msg.buttons[9]:# startでstart
#                 rospy.loginfo("start")
#                 self.pub_start_flag.publish()
                            
            
#             if msg.buttons[11]:# 右スティックで手動と自動切り替え
#                 self.isHandy = not self.isHandy
#                 self.pub_isHandy.publish(self.isHandy)
#                 if self.isHandy:
#                     rospy.loginfo("change auto -> hand")
#                 else :
#                     rospy.loginfo("change handy -> auto")
            
                
#     def update(self):
#         rospy.loginfo("enter main routine")
#         while not rospy.is_shutdown():
#             rospy.loginfo(self.left_axes)
#             rospy.loginfo(self.buttons)
            
            
#             if not self.is_emergence:               
#                 f_msg = Float32MultiArray(data=self.left_axes)
#                 self.pubLeftAxes.publish(f_msg)

#                 f_msg = Float32MultiArray(data=self.cross_axes)
#                 self.pubCross.publish(f_msg)

#                 if not len(self.buttons) == 0:
#                     if self.buttons[4] and not self.buttons[5]:
#                         rospy.loginfo("Left")
#                         f_msg = Float32(data = -1 * self.rot_ratio)
#                         self.pubRotate.publish(f_msg)
#                     elif self.buttons[5] and not self.buttons[4]:
#                         rospy.loginfo("Right")
#                         f_msg = Float32(data=self.rot_ratio)
#                         self.pubRotate.publish(f_msg)
                        
#                     if self.buttons[7] and not self.buttons[6]:
#                         rospy.loginfo("grab")
#                         self.pub_grab_cmd.publish()
#                     elif self.buttons[6] and not self.buttons[7]:
#                         rospy.loginfo("release")
#                         self.pub_release_cmd.publish()
                        
                    
#             self.r.sleep()

# if __name__ == '__main__':
#     try:
#         joy_con = JOY_CONTROLLER()
#     except:
#         rospy.loginfo("joy_controller : something wrong")
#     finally:
#         rospy.loginfo("joy_controller : end process")