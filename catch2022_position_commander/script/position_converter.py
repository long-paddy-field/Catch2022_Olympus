#!/usr/bin/env python3
# x-y座標での情報と2リンクの角度情報を相互変換
# 各アクチュエータへの指示を一括管理

import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32

class position_converter():
    def __init__(self,field_color):
        self.field = field_color
        
        self.pub_move_rad           = rospy.Publisher("move_rad",Float32MultiArray,queue_size = 100)             
        self.pub_servo_angle        = rospy.Publisher("servo_angle",Float32,queue_size=100)
        self.sub_move_cmd           = rospy.Subscriber("move_cmd",Float32MultiArray,self.move_cmd_callback,queue_size=100)
        self.sub_servo_cmd          = rospy.Subscriber("servo_cmd",Int8,self.servo_cmd_callback,queue_size=100)

        self.pub_current_position   = rospy.Publisher("current_position",Float32MultiArray,queue_size = 100)       
        self.sub_current_angle      = rospy.Subscriber("current_angle",Float32MultiArray,self.current_angle_callback,queue_size=100)
        
        self.sign                   = 1
        self.move_rad               = Float32MultiArray()
        self.servo_angle            = Float32()
        self.move_cmd               = Float32MultiArray()
        self.current_position       = Float32MultiArray()
        self.current_angle          = Float32MultiArray()
        
 
        rospy.loginfo(self.field)
        
        self.l1     = 0.6
        self.l2     = 0.3
        self.enable1 = False
        self.enable2 = False
        self.enable3 = False
        
        self.past_rad1 = 0
        
        self.r = rospy.Rate(100)
        self.update()
        
    def move_cmd_callback(self,msg):
        self.enable2 = True
        dist = math.sqrt(msg.data[0]**2+msg.data[1]**2)

        self.move_cmd.data = msg.data
        if  dist >= 0.89:
            self.move_cmd.data = (0.89 * self.move_cmd.data[0] / dist,0.89 * self.move_cmd.data[1] / dist)
        elif dist <= 0.31:
            self.move_cmd.data = (0.31 * self.move_cmd.data[0] / dist,0.31 * self.move_cmd.data[1] / dist)
        result = self.cartesian_to_rad(self.move_cmd.data[0],self.move_cmd.data[1])
        self.move_rad.data = [result[0],result[1]]
    
    def servo_cmd_callback(self,msg):
        # self.enable3 = True
        self.servo_angle.data = (msg.data+1) * (math.pi/2) - (self.current_angle.data[0]+self.current_angle.data[1])
        # if msg.data == True:
        #     self.servo_angle.data = -1 * (self.current_angle.data[0]+self.current_angle.data[1])
        # else :
        #     self.servo_angle.data = math.pi/2 - (self.current_angle.data[0]+self.current_angle.data[1])
            
        if self.servo_angle.data < 0:
            self.servo_angle.data += 2*math.pi
        elif self.servo_angle.data > math.pi*2:
            self.servo_angle.data -= 2*math.pi
    
    def current_angle_callback(self,msg):
        self.current_angle.data = msg.data
        # rospy.loginfo(self.current_angle.data)
        result = self.rad_to_cartesian(self.current_angle.data[0],self.current_angle.data[1])
        self.current_position.data = [result[0],result[1]]
        
        if not self.enable1:
            self.past_rad1 = result[0]
        self.enable1 = True

    
    def rad_to_cartesian(self, rad0,rad1):
        x = self.poi(self.l1 * math.cos(rad0) + self.l2*math.cos(rad0+rad1))
        y = self.poi(self.l1 * math.sin(rad0) + self.l2*math.sin(rad0+rad1))
        return x,y

    def cartesian_to_rad(self, x,y):
        if self.field == "red":
            rad1 = math.acos(((x**2)+(y**2)+(self.l1**2)-(self.l2**2))/(2*self.l1*math.sqrt(x**2+y**2)))+math.atan2(y,x)
        elif self.field == "blue":
            rad1 = (-1*math.acos(((x**2)+(y**2)+(self.l1**2)-(self.l2**2))/(2*self.l1*math.sqrt(x**2+y**2))))+math.atan2(y,x)
            
        rad2 = math.atan2(y-self.l1*math.sin(rad1),x-self.l1*math.cos(rad1))-rad1        
        return rad1,rad2
    
    def poi(self, arg: float):
        if math.fabs(arg) < 0.001:
            return 0
        else:
            return arg
    
    def update(self):
        while not rospy.is_shutdown():
            if self.enable1:
                self.pub_current_position.publish(self.current_position)
            if self.enable2:
                self.pub_move_rad.publish(self.move_rad)
                self.pub_servo_angle.publish(self.servo_angle)
                
            self.r.sleep()
            
    def my_atan(self,x,y):
        if x==0:
            if y > 0:
                return math.pi/2
            else:
                return -1 * math.pi/2
        else:
            return math.atan(y/x)
        
        
if __name__=='__main__':
    rospy.init_node("position_converter")
    field_color = rospy.get_param("~field_color")
    arg = position_converter(field_color)