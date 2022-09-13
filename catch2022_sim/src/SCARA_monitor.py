#!/usr/bin/env python3
import rospy
import math
from typing import List
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32


class simulator():
    def __init__(self, field_color):
        self.field = field_color

        self.pub_joint_states = rospy.Publisher("joint_states", JointState, queue_size=100)
        
        self.sub_current_angle = rospy.Subscriber("current_angle", Float32MultiArray,self.current_angle_callback, queue_size=100)
        self.sub_servo_angle = rospy.Subscriber("servo_angle", Float32, self.servo_angle_callback, queue_size=100)

        self.joint_states = JointState()
        self.joint_states.header = Header()
        self.joint_states.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']

        if self.field == "red":
            self.joint_states.position = [math.pi/6, -2*math.pi/3, 0, 0]
            self.current_angle = Float32MultiArray(data=[math.pi/6, -2*math.pi/3])
        elif self.field == "blue":
            self.joint_states.position = [-1*math.pi/6, 2*math.pi/3, 0, 0]
            self.current_angle = Float32MultiArray(data=[-1*math.pi/6, 2*math.pi/3])

        # self.current_angle = Float32MultiArray()
        self.servo_angle = Float32(data=0)

        self.r = rospy.Rate(100)
        self.update()

    def current_angle_callback(self, msg):
        self.current_angle.data = msg.data

    def servo_angle_callback(self, msg):
        self.servo_angle.data = msg.data

    def update(self):
        while not rospy.is_shutdown():
            self.joint_states.header.stamp = rospy.Time.now()
            self.joint_states.position = [self.current_angle.data[0], self.current_angle.data[1], 0, self.servo_angle.data]
            self.pub_joint_states.publish(self.joint_states)
            self.r.sleep()


if __name__ == "__main__":
    rospy.init_node("SCARA_monitor")
    field_color = rospy.get_param("~field_color")
    func = simulator(field_color)
    rospy.loginfo("SCARA_rviz : end process")