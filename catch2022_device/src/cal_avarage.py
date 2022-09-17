#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray

class CalAverage():
    def __init__(self):
        rospy.Subscriber("current_angle_raw",Float32MultiArray,self.current_angle_raw_callback,queue_size=1)
        self.pub_current_angle = rospy.Publisher("current_angle",Float32MultiArray,queue_size=100)

        self.data_queue0 = [0,0,0,0,0,0,0,0,0,0]
        self.data_sum0 = 0

        self.data_queue1 = [0,0,0,0,0,0,0,0,0,0]
        self.data_sum1 = 0
        self.my_counter = 0

        self.current_angle = Float32MultiArray(data=[0,0])
        
        self.r = rospy.Rate(10)
        self.update()

    def current_angle_raw_callback(self,msg):
        if self.my_counter < 10:
            self.data_queue0[self.my_counter] = msg.data[0]
            self.data_queue1[self.my_counter] = msg.data[1]
            self.data_sum0 += msg.data[0]
            self.data_sum1 += msg.data[1]
        else:
            self.data_sum0 = self.data_sum0 + msg.data[0] - self.data_queue0[0]
            self.data_sum1 = self.data_sum1 + msg.data[1] - self.data_queue1[0]
            for i in range(9):
                self.data_queue0[i] = self.data_queue0[i + 1]
                self.data_queue1[i] = self.data_queue1[i + 1]
            self.data_queue0[9] = msg.data[0]
            self.data_queue1[9] = msg.data[1]
        self.my_counter+=1

    def update(self):   
        while not rospy.is_shutdown():
            self.current_angle.data = [self.data_sum0/10,self.data_sum1/10]
            self.pub_current_angle.publish(self.current_angle)
            self.r.sleep()
        

if __name__ == '__main__':
    rospy.init_node("cal_avarage")
    arg = CalAverage()