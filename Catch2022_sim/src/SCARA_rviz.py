#!/usr/bin/env python3
import rospy

import math

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray


class rviz_simulator():
    def __init__(self):
        rospy.loginfo("SCARA_rviz : constructor activated")
        self.rviz_pub = rospy.Publisher("joint_states", JointState, queue_size=100)
        rospy.Subscriber("position_cmd",Float32MultiArray,self.cmd_callback,queue_size=100)
        
        self.rviz_msg = JointState()
        self.rate = rospy.Rate(10)
        self.rviz_cmd = []

        self.rviz_msg.header = Header()
        self.rviz_msg.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']
        rospy.loginfo("SCARA_rviz : registered names")

        
        self.rviz_msg.position = [0, 0, 0, 0]
        rospy.loginfo("SCARA_rviz : registered position")


        self.update()
    
    def cmd_callback(self,msg):
        self.rviz_cmd = msg.data
        self.rviz_msg.position = self.rviz_cmd
        

    def update(self):
        rospy.loginfo("SCARA_rviz : enter main routine")
        cnt = 0
        while not rospy.is_shutdown():
            # self.rviz_cmd = [cnt,cnt,cnt,cnt]
            # self.rviz_msg.position[0] = 1
            self.rviz_msg.header.stamp = rospy.Time.now()
            rospy.loginfo(self.rviz_msg.position)
            self.rviz_pub.publish(self.rviz_msg)
            cnt += 0.1
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("SCARA_rviz")
        func = rviz_simulator()
        pass
    except:
        rospy.loginfo("SCARA_rviz : something wrong")
        pass
    finally:
        rospy.loginfo("SCARA_rviz : end process")
        pass
