#!/usr/bin/env python3
#役割：ジョイコン入力の読み取り、各部への伝達（ボタン）

from faulthandler import is_enabled
import rospy
from std_msgs.msg       import Float32MultiArray
from std_msgs.msg       import Bool
from sensor_msgs.msg    import Joy
from std_msgs.msg       import Int8
from std_msgs.msg       import Empty

class btn_manager():
    def __init__(self):
        self.past_time = rospy.Time.now()
        self.current_time = rospy.Time.now()
        self.btn_flag = False
                
    def is_enabled(self,arg:Bool):
        if arg:
            self.current_time=rospy.Time.now()
            if self.current_time.secs - self.past_time.secs >= 0.75:
                self.past_time = rospy.Time.now()
                return True
            else:
                return False
        else:
            return False


class joy_controller():
    def __init__(self,field_color):
        self.field = field_color
        self.pub_move_cmd           = rospy.Publisher("move_cmd",Float32MultiArray,queue_size=100)
        self.pub_servo_cmd          = rospy.Publisher("servo_cmd",Int8,queue_size=100)
        self.pub_servo_cmd_ex       = rospy.Publisher("servo_cmd_ex",Int8,queue_size=100)
        self.pub_pmp_state          = rospy.Publisher("pmp_state",Int8,queue_size=100)
        self.pub_stepper_cmd        = rospy.Publisher("stepper_cmd",Bool,queue_size=100)
        self.pub_stepper_state      = rospy.Publisher("stepper_state",Int8,queue_size=100)

        self.pub_start_cmd          = rospy.Publisher("start_cmd",Empty,queue_size=100)
        self.pub_back_cmd           = rospy.Publisher("back_cmd",Empty,queue_size=100)
        self.pub_is_handy           = rospy.Publisher("is_handy",Bool, queue_size=100)
        self.pub_servo_enable       = rospy.Publisher("servo_enable",Empty,queue_size=100)
        
        self.pub_grab_cmd           = rospy.Publisher("grab_cmd",Empty,queue_size=100)
        self.pub_release_cmd        = rospy.Publisher("release_cmd",Empty,queue_size=100)
        self.pub_quick_release_cmd  = rospy.Publisher("quick_release_cmd",Empty,queue_size=100)
        
        self.sub_joy                = rospy.Subscriber("joy",Joy,self.joy_callback,queue_size=100)
        self.sub_current_position   = rospy.Subscriber("current_position",Float32MultiArray,self.current_position_callback,queue_size=100)
        self.sub_servo_cmd          = rospy.Subscriber("servo_cmd",Int8,self.servo_cmd_callback,queue_size=100)
        
        self.pub_task_selector      = rospy.Publisher("task_selector",Int8,queue_size=100)
        
        self.delta_x = 0
        self.delta_y = 0
        
        self.current_x = 0
        self.current_y = 0
        self.move_cmd = Float32MultiArray()
        self.buttons = list()
        
        self.servo_cmd = Int8(data = 0)
        self.pmp_state = Int8(data = 0)
        self.is_handy  = Bool(data = True)
        self.stepper_cmd = Bool(data = True)
        self.stepper_state = Int8(data = 8)
        self.servo_cmd_ex = Int8(data = 0)
        self.task_shift = Int8(data = 0)

        self.enable = False
        
        self.btn0  = btn_manager()
        self.btn1  = btn_manager()
        self.btn2  = btn_manager()
        self.btn3  = btn_manager()
        self.btn4  = btn_manager()
        self.btn5  = btn_manager()
        self.btn6  = btn_manager()
        self.btn7  = btn_manager()
        self.btn8  = btn_manager()
        self.btn9  = btn_manager()
        self.btn10 = btn_manager()
        self.btn11 = btn_manager()
        self.r = rospy.Rate(10)
        self.update()
        
    def joy_callback(self,msg):
        if self.field == "blue":
            self.delta_x = -0.1*msg.axes[0] - 0.05*msg.axes[4]
            self.delta_y =  0.1*msg.axes[1] + 0.05*msg.axes[5]

        elif self.field == "red":
            self.delta_x =  0.1*msg.axes[0] + 0.05*msg.axes[4]
            self.delta_y = -0.1*msg.axes[1] - 0.05*msg.axes[5]
        # rospy.loginfo("%f,%f",self.delta_x,self.delta_y)
        self.buttons = msg.buttons

    
    def current_position_callback(self,msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.enable = True
    
    def servo_cmd_callback(self,msg):
        if not self.is_handy:
            self.servo_cmd.data = msg.data
    
    def update(self):
        while not rospy.is_shutdown():
            # rospy.loginfo(self.buttons)
            if self.enable and not len(self.buttons) == 0:
                self.move_cmd.data = [self.current_x+self.delta_x,self.current_y+self.delta_y]
                rospy.loginfo(self.is_handy)

                if self.is_handy.data:
                    if self.btn3.is_enabled(self.buttons[3]):
                        self.pub_quick_release_cmd.publish()
                    # if self.btn0.is_enabled(self.buttons[0]):     #青シール側の把持のみ操作
                    #     if self.pmp_state.data < 2 :
                    #         self.pmp_state.data += 2
                    #         self.pub_grab_cmd.publish()
                    #     else:
                    #         self.pmp_state.data -= 2
                    #         self.pub_release_cmd.publish()
                    # elif self.btn2.is_enabled(self.buttons[2]):   #赤シール側の把持のみ操作
                    #     if self.pmp_state.data % 2 == 0:
                    #         self.pmp_state.data += 1
                    #         self.pub_grab_cmd.publish()
                    #     else:
                    #         self.pmp_state.data -= 1
                    #         self.pub_release_cmd.publish()
                    
                    # if self.btn1.is_enabled(self.buttons[1]):     #ワークはなす
                    #     self.pmp_state.data = 0
                    # elif self.btn3.is_enabled(self.buttons[3]):   #ワーク掴む
                    #     self.pmp_state.data = 3
                        
                    if self.btn4.is_enabled(self.buttons[4]):     #サーボCCW
                        self.servo_cmd.data += 1
                        self.servo_cmd_ex = 1
                    elif self.btn5.is_enabled(self.buttons[5]):   #サーボCW
                        self.servo_cmd.data -= 1
                        self.servo_cmd_ex = -1
                    # if self.btn6.is_enabled(self.buttons[6]):     #ステッパ下降
                    #     self.stepper_cmd.data = False
                    #     self.pub_stepper_cmd.publish(self.stepper_cmd)
                    # elif self.btn7.is_enabled(self.buttons[7]):   #ステッパ上昇
                    #     self.stepper_cmd.data = True
                    #     self.pub_stepper_cmd.publish(self.stepper_cmd)
                    self.pub_move_cmd.publish(self.move_cmd)     
                    rospy.loginfo("nya")           
                    # self.pub_servo_cmd.publish(self.servo_cmd)
                    self.pub_servo_cmd_ex.publish(self.servo_cmd_ex)
                    # rospy.loginfo("joyjoy")
                    # self.pub_pmp_state.publish(self.pmp_state)

                if self.btn0.is_enabled(self.buttons[0]):
                    self.task_shift = -1
                elif self.btn2.is_enabled(self.buttons[2]):
                    self.task_shift = 1
                else:
                    self.task_shift = 0

                if self.btn6.is_enabled(self.buttons[6]):  # サーボ許可
                    self.pub_servo_enable.publish()
                if self.btn7.is_enabled(self.buttons[7]):
                    self.stepper_state.data = 8
                    self.pub_stepper_state.publish(self.stepper_state)
                    rospy.sleep(1)
                    self.stepper_state.data = 0
                    self.pub_stepper_state.publish(self.stepper_state)
                if self.btn8.is_enabled(self.buttons[8]):
                    self.pub_back_cmd.publish()
                                        
                if self.btn11.is_enabled(self.buttons[11]):    #手動自動切り替え
                    if self.is_handy.data:
                        rospy.loginfo("hand -> auto")
                    else :
                        rospy.loginfo("auto -> hand")
                    self.is_handy.data = not self.is_handy.data
                    
                self.pub_is_handy.publish(self.is_handy)
                # rospy.loginfo(self.is_handy)
            if not len(self.buttons) == 0:
                if self.btn9.is_enabled(self.buttons[9]):  # START
                    self.pub_start_cmd.publish()

            self.r.sleep()
            
    
if __name__ == '__main__':
    rospy.init_node('joy_controller')
    field_color = rospy.get_param("~field_color")
    # field_color = "red"
    rospy.loginfo("node is activated")
    arg = joy_controller(field_color)
    rospy.loginfo("joy_controller : process_end")