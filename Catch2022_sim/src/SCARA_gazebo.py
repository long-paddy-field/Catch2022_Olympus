#!/usr/bin/env python3
from numpy import float64
from rosdep2 import RosdepInternalError
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
import math

rviz_cmd=[0,0,0,0]


def rviz_callback(data):
    global rviz_cmd    
    rviz_cmd = data



def simulator():
    global rviz_cmd
    rospy.init_node("SCARA_sim")
    rospy.Subscriber("motor_commands",Float32MultiArray,rviz_callback,queue_size=100)
    rviz_pub = rospy.Publisher("joint_states",JointState,queue_size=100)
    gazebo_arm1_pub = rospy.Publisher("my_SCARA/arm1_position_controller/command",Float64,queue_size=100)
    gazebo_arm2_pub = rospy.Publisher("my_SCARA/arm2_position_controller/command",Float64,queue_size=100)
    gazebo_linear_pub = rospy.Publisher("my_SCARA/linear_position_controller/command",Float64,queue_size=100)
    gazebo_wrist_pub = rospy.Publisher("my_SCARA/wrist_position_controller/command",Float64,queue_size=100)
    

    r = rospy.Rate(10)

    rviz_msg = JointState()

    rviz_msg.name[0] = "stand_arm1"
    rviz_msg.name[1] = "arm1_arm2"
    rviz_msg.name[2] = "arm2_linear"
    rviz_msg.name[3] = "linear_wrist"
    rospy.loginfo("enter main routine")
    while not rospy.is_shutdown():
        rviz_msg.header.stamp = rospy.Time.now()
        rviz_msg.position = rviz_cmd
        rospy.loginfo("published")     
        rviz_pub.publish(rviz_msg)
        r.sleep()

if __name__ == "__main__" :
    try:
        simulator()
    
    except RosdepInternalError: pass