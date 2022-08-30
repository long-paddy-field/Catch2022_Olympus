#!/usr/bin/env python3

import queue
import rospy
from std_msgs.msg import Float32MultiArray

class test():
    def __init__(self):
        rospy.init_node("test")
        self.r = rospy.Rate(10)
        self.pub_current_angle = rospy.Publisher("current_angle",Float32MultiArray,queue_size=100)
        self.pub_target_location = rospy.Publisher("target_location",Float32MultiArray,queue_size=100)
        self.my_msg =Float32MultiArray(data = [95,258])
        rospy.Subscriber("move_cmd",Float32MultiArray,self.move_cmd_callback,queue_size=100)

        self.update()

    def move_cmd_callback(self,msg):
        self.my_msg.data = msg.data
        rospy.loginfo(msg)

    def update(self):

        while not rospy.is_shutdown():
            self.pub_current_angle.publish(self.my_msg)
            f_msg = Float32MultiArray()
            f_msg.data = [0.3,0.82]
            rospy.loginfo("pub")
            self.pub_target_location.publish(f_msg)

            self.r.sleep()


if __name__ == '__main__':
    my_test=test()
