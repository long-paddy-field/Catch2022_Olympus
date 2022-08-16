#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('device')
    pub=rospy.Publisher('device',String,queue_size=10)
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
        msg="hello, world!"
        pub.publish(msg)
        rate.sleep()



if __name__=="__main__":
    main()