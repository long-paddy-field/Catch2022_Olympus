#!/usr/bin/env python3
#役割：ジョイコン入力の読み取り、各部への伝達（ボタン）

import rospy
from std_msgs.msg       import Float32MultiArray
from std_msgs.msg       import Bool
from sensor_msgs.msg    import Joy

class joy_controller():
    def __init__(self,field_color):
        self.field = field_color
        self.pub_move_cmd           = rospy.Publisher("move_cmd",Float32MultiArray,queue_size=100)
        self.pub_servo_cmd          = rospy.Publisher("servo_cmd",Bool,queue_size=100)
        
        self.sub_joy                = rospy.Subscriber("joy",Joy,self.joy_callback,queue_size=100)
        self.sub_current_position   = rospy.Subscriber("current_position",Float32MultiArray,self.current_position_callback,queue_size=100)
        
        self.delta_x = 0
        self.delta_y = 0
        
        self.current_x = 0
        self.current_y = 0
        self.move_cmd = Float32MultiArray()
        self.buttons = list()
        
        self.servo_cmd = Bool(data = True)
        
        self.enable = False
        
        self.r = rospy.Rate(10)
        self.update()
        
    def joy_callback(self,msg):
        if self.field == "blue":
            self.delta_x = -0.01*msg.axes[0] - 0.0005*msg.axes[4]
            self.delta_y =  0.01*msg.axes[1] + 0.0005*msg.axes[5]
        elif self.field == "red":
            self.delta_x =  0.01*msg.axes[0] + 0.0005*msg.axes[4]
            self.delta_y = -0.01*msg.axes[1] - 0.0005*msg.axes[5]
            
        self.buttons = msg.buttons

    
    def current_position_callback(self,msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.enable = True
    
    def update(self):
        while not rospy.is_shutdown():
            if self.enable and not len(self.buttons) == 0:
                self.move_cmd.data = [self.current_x+self.delta_x,self.current_y+self.delta_y]
                
                if self.buttons[10]:    #サーボの向き変更
                    self.servo_cmd.data = not self.servo_cmd.data

                self.pub_move_cmd.publish(self.move_cmd)                
                self.pub_servo_cmd.publish(self.servo_cmd)
            self.r.sleep()
            
    
if __name__ == '__main__':
    rospy.init_node('joy_controller')
    field_color = rospy.get_param("~field_color")
    arg = joy_controller(field_color)
    rospy.loginfo("joy_controller : process_end")

# import queue
# import rospy
# from sensor_msgs.msg import Joy
# from std_msgs.msg import String
# from std_msgs.msg import Empty
# from std_msgs.msg import Float32MultiArray
# from std_msgs.msg import Int8
# from std_msgs.msg import Bool
# from std_msgs.msg import Float32

# class JOY_CONTROLLER():
#     def __init__(self):
#         rospy.loginfo("joy_controller : start process")
#         self.initializer()
#         self.update()
        
#     def initializer(self):
#         self.r = rospy.Rate(10)
        
#         self.pub_move_cmd       = rospy.Publisher("move_cmd",Float32MultiArray,queue_size = 100)
#         self.pub_is_blue        = rospy.Publisher("is_blue",Bool,queue_size = 100)       # フィールドの色を設定
#         self.pub_pmp_state      = rospy.Publisher("pmp_state",Bool,queue_size = 100)     # 真で吸う
#         self.pub_grab_cmd       = rospy.Publisher("grab_cmd",Empty,queue_size = 100)     # 掴む
#         self.pub_release_cmd    = rospy.Publisher("release_cmd",Empty,queue_size = 100)  # 離す
#         self.pub_servo_cmd      = rospy.Publisher("servo_cmd",Bool,queue_size = 100)     # 0 でまっすぐ 1 で垂直
#         self.pub_step_cmd       = rospy.Publisher("step_cmd",Bool,queue_size = 100)      # ステッパ 1 が 上昇
#         self.pub_emergence_cmd  = rospy.Publisher("emergence_cmd",Int8,queue_size = 100) # 緊急停止
#         self.pub_start_flag     = rospy.Publisher("start_flag",Empty,queue_size = 100)   # スタート
#         self.pub_is_Handy       = rospy.Publisher("is_handy",Bool,queue_size = 100)      # 手動自動切り替え
        
#         rospy.Subscriber("joy", Joy,self.joy_callback,queue_size = 100)
#         rospy.Subscriber("current_position",Float32MultiArray,self.current_position_callback,queue_size=100)
        
#         self.buttons        = list()
#         self.is_blue        = Bool()
#         self.servo_cmd      = Bool()
#         self.step_cmd       = Int8()
#         self.emergence_cmd  = Int8()
#         self.is_Handy       = Bool()
#         self.move_cmd       = Float32MultiArray()
        
#         self.current_position = [1,1]
#         self.next_position  = [0,0]        
#         self.is_blue.data       = False
#         self.servo_cmd.data     = False
#         self.step_cmd.data      = 0
#         self.emergence_cmd      = 0xff
#         self.is_Handy           = False
#         self.emergence          = False
#         rospy.loginfo("joy_controller : end setting")
        
#     def joy_callback(self,msg):
#         if not len(self.current_position) == 0:

#             self.next_position  = [self.current_position[0] - 0.01*msg.axes[0] - 0.0005*msg.axes[4],
#                                 self.current_position[1] + 0.01*msg.axes[1] + 0.0005*msg.axes[5]]
#         else :
#             self.next_position  = [- 0.01*msg.axes[0] - 0.0005*msg.axes[4],
#                                  + 0.01*msg.axes[1] + 0.0005*msg.axes[5]]
#         rospy.loginfo(self.next_position)
#         self.right_axes     = [-1*msg.axes[2],msg.axes[3]]
#         self.buttons        = msg.buttons
        
#     def current_position_callback(self,msg):
#         self.current_position = msg.data
        
#     def update(self):

#         while not rospy.is_shutdown():
#             if not self.emergence and not len(self.buttons) == 0:
#                 if self.is_Handy:
#                     f_msg = Float32MultiArray(data=self.move_cmd)
#                     self.pub_move_cmd.publish(f_msg)

#                     if self.buttons[0] and not self.buttons[2]: #青サイドのとき押す
#                         self.is_blue.data = True
#                         self.pub_is_blue.publish(self.is_blue)
                    
#                     if self.buttons[1] and not self.buttons[3]: #掴む
#                         self.pub_grab_cmd.publish()
#                         b_msg = Bool(data = True)
#                         self.pub_pmp_state.publish(b_msg)
                        
#                     if self.buttons[2] and not self.buttons[0]: #赤サイドのとき押す
#                         self.is_blue.data = False
#                         self.pub_is_blue.publish(self.is_blue)
                        
#                     if self.buttons[3] and not self.buttons[1]: #離す
#                         self.pub_release_cmd.publish()
#                         b_msg = Bool(data = False)
#                         self.pub_pmp_state.publish(b_msg)
                                                
#                     if self.buttons[6] and not self.buttons[7]: #ステップ上昇
#                         self.step_cmd.data = 1
#                         self.pub_step_cmd.publish(self.step_cmd)
                        
#                     if self.buttons[7] and not self.buttons[6]: #ステップ下降
#                         self.step_cmd.data = 0
#                         self.pub_step_cmd.publish(self.step_cmd)
                
#                     if self.buttons[10]:                        #サーボ駆動
#                         self.servo_cmd = not self.servo_cmd
#                         self.pub_servo_cmd.publish(self.servo_cmd)
                
#                 if self.buttons[9]:                             #動作開始
#                     self.pub_start_flag.publish()
                
#                 if self.buttons[11]:                            #自動手動切り替え
#                     self.is_Handy = not self.is_Handy               
#                     if self.is_Handy:
#                         rospy.loginfo("change auto -> hand")
#                     else :
#                         rospy.loginfo("change handy -> auto")
#                 self.pub_is_Handy.publish(self.is_Handy)
                        
#             if not len(self.buttons) == 0 and self.buttons[8]:   #緊急停止
#                 self.pub_emergence_cmd.publish(self.emergence_cmd)
#                 self.emergence = not self.emergence            
            
#             self.r.sleep()
        

# if __name__ == '__main__':
#     rospy.init_node('joy_controller')
#     joy_con = JOY_CONTROLLER()
#     rospy.loginfo("joy_controller : end process")