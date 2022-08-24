#!/usr/bin/env python3
from email.header import Header
from shutil import move
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from cobs import cobs
import serial
import struct

port = "/dev/ttyUSB0"


class device():
    def __init__(self):
        self.setup()
        self.loop()

    def move_cmd_callback(self,msg):
        self.move_cmd = msg.data
        # rospy.loginfo(uart_msg)
        # motor=struct.pack('<ff',*move_cmd)
        # self.uart.write(motor)
        # rospy.loginfo(motor)


    def servo_angle_callback(self,msg):
        self.servo_angle=msg.data


    def stepper_state_callback(self,msg):
        self.stepper_state=msg.data.to_bytes(1,'little')

    def pump_state_callback(self,msg):
        self.pump_state=msg.data

    def emergency_callback(self,msg):
        self.emergency=msg.data

    def setup(self):
        global port
        self.uart = serial.Serial(port, 115200)
        self.pub0 = rospy.Publisher('current_angle', JointState, queue_size=1)
        self.pub1 = rospy.Publisher(
            'is_grabbed', Int32MultiArray, queue_size=1)
        self.rate = rospy.Rate(100)

        # subscriberの宣言
        self.sub_move_cmd = rospy.Subscriber(
            'move_cmd', Float32MultiArray, self.move_cmd_callback, queue_size=1)
        self.sub_servo_angle = rospy.Subscriber(
            'servo_angle', Float32, self.servo_angle_callback, queue_size=1)
        self.sub_stepper_state = rospy.Subscriber(
            'stepper_state', Int8, self.stepper_state_callback, queue_size=1)
        self.sub_pump_state = rospy.Subscriber(
            'pump_state', Bool, self.pump_state_callback, queue_size=1)
        self.sub_emergency = rospy.Subscriber(
            'emergency', Int8, self.emergency_callback, queue_size=1)

        self.msg = Float32MultiArray(data=[1, 2])
        
        #変数の初期化
        self.move_cmd=[0x00,0x00]
        self.servo_angle=0x00
        self.stepper_state=b'\x00'
        self.pump_state=b'\x00'
        self.emergency=b'\x00'

    def loop(self):
        while not rospy.is_shutdown():
            # rospy.loginfo("hello,world")
            # self.sendSerial()
            # uart.write(message.encode('ascii'))
            # uart_msg=struct.pack("<fffc??c",*self.move_cmd,self.servo_angle,self.stepper_state,self.pump_state,self.emergency,0xFF)
            uart_msg=struct.pack("<fffc??c",*self.move_cmd,self.servo_angle,self.stepper_state,self.pump_state,self.emergency,b'\xFF')
            rospy.loginfo(uart_msg)
            self.uart.write(uart_msg)
            self.rate.sleep()

    def sendSerial():
        pass

    def receiveSerial(self):
        # 受信と整形
        currentAngle = JointState()
        currentAngle.header = Header()
        currentAngle.name = ['stand_arm1', 'arm1_arm2']
        currentAngle.position = [0, 0]  # 角度換算して入れる
        isGrabbed = Int32MultiArray(data=[1, 1])
        self.pub0.publish(currentAngle)
        self.pub1.publish(isGrabbed)


if __name__ == "__main__":
    # try:
    rospy.init_node('device')
    rospy.loginfo("device : node is activated")
    device = device()
    # except:
    #     rospy.loginfo("device : something wrong")
    # finally:
    #     rospy.loginfo("device : process end")
