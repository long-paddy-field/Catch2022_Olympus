#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header
from std_msgs.msg       import Float32MultiArray
from std_msgs.msg       import Float32


class simulator():
    def __init__(self,field_color):
        self.field = field_color
        
        self.pub_joint_states    = rospy.Publisher("joint_states",JointState,queue_size = 100)
        self.pub_current_angle   = rospy.Publisher("current_angle",Float32MultiArray,queue_size = 100)
        
        self.sub_move_rad        = rospy.Subscriber("move_rad",Float32MultiArray,self.move_rad_callback,queue_size=100)
        self.sub_servo_angle     = rospy.Subscriber("servo_angle",Float32,self.servo_angle_callback,queue_size=100)
        
        self.joint_states        = JointState()
        self.joint_states.header = Header()
        self.joint_states.name   = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']
        
        if self.field == "red":
            self.joint_states.position = [math.pi/6,-2*math.pi/3,0,0]
            self.current_angle       = Float32MultiArray(data=[math.pi/6,-2*math.pi/3])
        elif self.field == "blue":
            self.joint_states.position = [-1*math.pi/6,2*math.pi/3,0,0]
            self.current_angle       = Float32MultiArray(data=[-1*math.pi/6,2*math.pi/3])
        
        
        self.move_rad            = Float32MultiArray()
        self.servo_angle         = Float32(data = 0) 

        self.r = rospy.Rate(100)
        self.update()
        
        
    def move_rad_callback(self,msg):
        self.move_rad.data = msg.data
        self.current_angle.data = msg.data
    
    def servo_angle_callback(self,msg):
        self.servo_angle.data = msg.data
    
    def update(self):
        while not rospy.is_shutdown():
            self.joint_states.header.stamp = rospy.Time.now()
            self.joint_states.position = [self.current_angle.data[0],self.current_angle.data[1],0,self.servo_angle.data]
            self.pub_current_angle.publish(self.current_angle)
            self.pub_joint_states.publish(self.joint_states)
            self.r.sleep()

if __name__ == "__main__":
    rospy.init_node("SCARA_rviz")
    field_color = rospy.get_param("~field_color")
    func = simulator(field_color)
    rospy.loginfo("SCARA_rviz : end process")

# class rviz_simulator():
#     def __init__(self):
#         self.pub_joint_states       = rospy.Publisher("joint_states", JointState, queue_size=100)
#         self.pub_current_angle      = rospy.Publisher('current_angle', Float32MultiArray, queue_size=1)

#         self.sub_move_rad           = rospy.Subscriber('move_rad', Float32MultiArray, self.move_rad_callback, queue_size=100)
#         self.sub_servo_angle        = rospy.Subscriber('servo_angle', Float32MultiArray, self.servo_angle_callback, queue_size=100)
        
#         self.rviz_msg = JointState()
#         self.rviz_msg.header = Header()
#         self.rviz_msg.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']       
#         self.rviz_msg.position = [-2*math.pi/3, 2*math.pi/3, 0, 0]
        
#         self.rate = rospy.Rate(100)

#         self.current_position = Float32MultiArray(data=[])
#         self.l1 = 0.6
#         self.l2 = 0.3
#         self.sign = 1
#         self.task_num = 1
#         self.update()
    
#     def servo_angle_callback(self, msg):
#         self.servo_cmd = msg.data
#         if msg.data == True:
#             self.servo_angle = -(self.servo_angle[0] + self.servo_angle[1])
#         elif msg.data == False:
#             self.servo_angle = math.pi/2 - (self.servo_angle[0] + self.servo_angle[1])
        
#         if self.servo_angle < 0:
#             self.servo_angle = self.servo_angle + math.pi
#         elif self.servo_angle > 2*math.pi:
#             self.servo_angle = self.servo_angle - math.pi
            
#         self.rviz_msg.position[2] = math.pi*self.servo_angle/180    
    
#     def move_rad_callback(self,msg):
#         self.rviz_cmd_rad = self.cartesian_to_rad(msg.data)
#         self.rviz_msg.position = [self.rviz_cmd_rad[0],self.rviz_cmd_rad[1],0,0]
#         self.current_position.data = msg.data
        
#     def current_state_callback(self,msg):
#         self.task_num = msg.data    

#     def update(self):
#         rospy.loginfo("SCARA_rviz : enter main routine")
#         while not rospy.is_shutdown():
#             self.rviz_msg.header.stamp = rospy.Time.now()
            
#             rospy.loginfo(self.rviz_msg.position)
#             self.pub_joint_states.publish(self.rviz_msg)
#             self.pub_current_position.publish(self.current_position)
#             self.rate.sleep()


# if __name__ == "__main__":
#     rospy.init_node("SCARA_rviz")
#     func = rviz_simulator()
#     rospy.loginfo("SCARA_rviz : end process")