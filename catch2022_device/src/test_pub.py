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
    # uart=serial.Serial(port,115200)

    pub0 = rospy.Publisher('move_cmd', Float32MultiArray, queue_size=1)
    pub1 = rospy.Publisher('servo_angle', Float32, queue_size=1)
    pub2 = rospy.Publisher('stepper_state', Int8, queue_size=1)
    pub3 = rospy.Publisher('pump_state', Bool, queue_size=1)
    pub4 = rospy.Publisher('emergency', Int8, queue_size=1)
    
    rate = rospy.Rate(100)

    move_cmd = Float32MultiArray(data=[40, 125])
    servo_angle=Float32(data=90)
    stepper_state=Int8(data=3)
    pmp_state=Bool(data=1)
    emergency=Int8(data=0)
    pub1.publish(servo_angle)
    while not rospy.is_shutdown():
        pub0.publish(move_cmd)
        # rospy.loginfo(msg.data)
        # pub1.publish(servo_angle)
        pub2.publish(stepper_state)
        pub3.publish(pmp_state)
        pub4.publish(emergency)
        
        # message = 'Test/ message from pyserial\n'
        # uart.write(message.encode('ascii'))

        # rospy.loginfo("hello,world")
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