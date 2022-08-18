#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
import serial

port="/dev/pts/13"

def main():
    global port
    rospy.init_node('test_pub')
    rospy.loginfo("hello,world")
    uart=serial.Serial(port,115200)

    # pub0 = rospy.Publisher('move_cmd', Float32MultiArray, queue_size=1)
    # pub1 = rospy.Publisher('servo_angle', Float32, queue_size=1)
    # pub2 = rospy.Publisher('stepper_state', Int8, queue_size=1)
    # pub3 = rospy.Publisher('pump_state', Bool, queue_size=1)
    # pub4 = rospy.Publisher('emergency', Empty, queue_size=1)
    
    rate = rospy.Rate(100)

    msg = Float32MultiArray(data=[1, 2])

    while not rospy.is_shutdown():
        # pub0.publish(msg)
        # pub1.publish(msg)
        # pub2.publish(msg)
        # pub3.publish(msg)
        
        message = 'Test message from pyserial\n'
        uart.write(message.encode('ascii'))

        rospy.loginfo("hello,world")
        rate.sleep()

if __name__=="__main__":
    main()
    # try:
    #     rospy.loginfo("test_pub : node is activated")
    #     main()
    # except:
    #     rospy.loginfo("test_pub : something wrong")
    # finally:
    #     rospy.loginfo("test_pub : process end")