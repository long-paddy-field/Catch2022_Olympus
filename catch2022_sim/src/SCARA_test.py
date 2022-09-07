#!/usr/bin/env python3
import rospy
import tf2_ros
from std_msgs.msg import Float32MultiArray

class SCARA_test:
    def __init__(self):
        rospy.loginfo("SCARA_test : constructor activated")
        self.test_pub = rospy.Publisher("position_cmd",Float32MultiArray,queue_size=100)
        
        self.rate = rospy.Rate(10)
        self.test_msg = Float32MultiArray()
        self.test_msg.data = [1, 0, 1, 1]
        self.update()
        
    def update(self):
        rospy.loginfo("SCARA_test : enter main routine")
        while not rospy.is_shutdown():
            rospy.loginfo(self.test_msg.data)
            self.test_pub.publish(self.test_msg)
            self.rate.sleep()
            
            

if __name__ == "__main__":
    try:
        rospy.init_node("SCARA_test")
        func = SCARA_test()
        pass
    except:
        print("SCARA_test : something wrong")
    finally:
        print("SCARA_test : process end")    
    