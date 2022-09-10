#!/usr/bin/env python3

from std_msgs.msg import Empty
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from math import pi
import math

class Scara():
    def __init__(self):
        self.work_flag = False
        self.l1     = 0.6
        self.l2     = 0.3
        self.a      = 1
        self.h      = 10
        self.r      = rospy.Rate(10)
        self.target = np.array([[0],[0]])
        self.current = np.array([[0],[0.52]])
        self.current_theta = np.array([[pi/3],[pi/1.5]])
        self.next_r = np.array([[0],[0]])
        self.start = np.array([[0],[0]])
        self.J=np.array([[1,0],[0,1]])
        self.is_handy = True
        
        self.pub_move_cmd = rospy.Publisher("move_cmd",Float32MultiArray,queue_size = 100)
        self.pub_end_cmd = rospy.Publisher("end_cmd",Empty,queue_size= 100)
        rospy.Subscriber("current_position",Float32MultiArray,self.current_position_callback,queue_size = 100)
        rospy.Subscriber("target_location",Float32MultiArray,self.target_location_callback,queue_size = 100)
        rospy.Subscriber("is_handy",Bool,self.is_handy_callback,queue_size=100)
        self.update()
                

    def target_location_callback(self,msg):
        self.target = np.array([[msg.data[0]],[msg.data[1]]])
        self.work_flag = True
        self.er = self.target - self.start
        self.distance=np.linalg.norm(self.er)
        self.t0 = math.sqrt(self.distance/self.a)
        rospy.loginfo(self.t0)
        
    def current_position_callback(self,msg):
        self.current_x = msg.data[0]
        self.current_y = msg.data[1]
        self.current = np.array([[self.current_x],[self.current_y]])
        if not self.work_flag:
            self.start = self.current
    
    def is_handy_callback(self,msg):
        self.is_handy = msg.data

    def update(self):
        cnt = 1
        while not rospy.is_shutdown():
            if not self.is_handy:
                if self.work_flag == True:
                    if cnt <= self.h * self.t0:
                        next_dist = self.a*(2*cnt-1)/(2*(self.h**2))
                        self.next_r = (next_dist/self.distance)*self.er
                        cnt = cnt + 1
                        self.next_pos =  (self.next_r + self.current)
                        f_msg = Float32MultiArray(data = [self.next_pos[0,0],self.next_pos[1,0]])
                        self.pub_move_cmd.publish(f_msg)    
                    elif cnt <= self.h * self.t0*2:
                        next_dist = (self.a/(2*self.h))*(4*self.t0-((2*cnt-1)/self.h))
                        self.next_r = (next_dist/self.distance)*self.er    
                        cnt = cnt + 1
                        self.next_pos =  (self.next_r + self.current)
                        f_msg = Float32MultiArray(data = [self.next_pos[0,0],self.next_pos[1,0]])
                        self.pub_move_cmd.publish(f_msg)    
                    else:
                        self.work_flag = False
                        cnt = 1
                        self.pub_move_cmd.publish(data= [self.target[0,0],self.target[1,0]])
                        rospy.loginfo("scara_ik ended")
                        self.pub_end_cmd.publish()
                        
            self.r.sleep()

if __name__ == "__main__":
    rospy.init_node("scara_ik")
    rospy.loginfo("scara_ik : node is activated")
    scara =Scara()
    rospy.loginfo("scara_ik : process_end")