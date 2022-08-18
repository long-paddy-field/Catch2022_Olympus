#!/usr/bin/env python3
from email.header import Header
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState


class device():
    def __init__(self):
        self.setup()
        self.loop()

    def move_cmd_callback(msg):
        testmsg = msg.data
        # rospy.loginfo(testmsg[1])

    def setup(self):
        self.pub0 = rospy.Publisher('current_angle', JointState, queue_size=1)
        self.pub1 = rospy.Publisher('is_grabbed', Int32MultiArray, queue_size=1)
        self.rate = rospy.Rate(100)
        self.sub = rospy.Subscriber('move_cmd', Float32MultiArray,self.move_cmd_callback, queue_size=1)
        self.msg = Float32MultiArray(data=[1, 2])


    def loop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("hello,world")
            # self.sendSerial()
            self.rate.sleep()

    def sendSerial():
        pass


    def receiveSerial(self):
        #受信と整形
        currentAngle = JointState()
        currentAngle.header = Header()
        currentAngle.name = ['stand_arm1', 'arm1_arm2']
        currentAngle.position = [0, 0]  #角度換算して入れる
        isGrabbed=Int32MultiArray(data=[1,1])
        self.pub0.publish(currentAngle)
        self.pub1.publish(isGrabbed)



if __name__ == "__main__":
    try:
        rospy.init_node('device')
        rospy.loginfo("device : node is activated")
        device=device()
    except:
        rospy.loginfo("device : something wrong")
    finally:
        rospy.loginfo("device : process end")
