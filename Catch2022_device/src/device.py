#!/usr/bin/env python3
from cmath import pi
from email.header import Header
from shutil import move
from time import struct_time
import rospy
from typing import List
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Header
from cobs import cobs
import serial
import struct
import math
import serial.tools.list_ports

port = serial.tools.list_ports.comports()[0].device

class device():

    def __init__(self):
        self.setup()
        self.loop()

    def move_cmd_callback(self, msg):
        self.move_cmd = msg.data
        self.move_cmd_theta = self.cartesian_to_theta(self.move_cmd)
        # rospy.loginfo(uart_msg)
        # motor=struct.pack('<ff',*move_cmd)
        # self.uart.write(motor)
        # rospy.loginfo(motor)

    def servo_angle_callback(self, msg):
        self.servo_angle = msg.data

    def stepper_state_callback(self, msg):
        self.stepper_state = msg.data.to_bytes(1, 'little')

    def pump_state_callback(self, msg):
        self.pump_state = msg.data

    def emergency_callback(self, msg):
        self.emergency = msg.data

    def is_blue_callback(self, msg):
        if msg.data == True:
            self.sign = -1
        else:
            self.sign = 1

    def setup(self):
        global port
        self.uart = serial.Serial(port, 115200)
        self.pub0 = rospy.Publisher('current_angle', Float32MultiArray, queue_size=1)
        self.pub1 = rospy.Publisher('is_grabbed', Int8MultiArray, queue_size=1)
        self.pub2 = rospy.Publisher('current_position',Float32MultiArray,queue_size=1)
        self.rate = rospy.Rate(100)
        self.l1 = 0.6
        self.l2 = 0.3
        self.sign = 1

        # subscriberの宣言
        self.sub_move_cmd       = rospy.Subscriber('move_cmd', Float32MultiArray, self.move_cmd_callback, queue_size=1)
        self.sub_servo_angle    = rospy.Subscriber('servo_angle', Float32, self.servo_angle_callback, queue_size=1)
        self.sub_stepper_state  = rospy.Subscriber('stepper_state', Int8, self.stepper_state_callback, queue_size=1)
        self.sub_pump_state     = rospy.Subscriber('pump_state', Bool, self.pump_state_callback, queue_size=1)
        self.sub_emergency      = rospy.Subscriber('emergency', Int8, self.emergency_callback, queue_size=1)
        self.sub_color_field    = rospy.Subscriber('is_blue', Bool, self.is_blue_callback, queue_size=100)
        self.msg                = Float32MultiArray(data=[1, 2])

        # 変数の初期化
        self.move_cmd = [90,78]
        self.servo_angle = 0x00
        self.stepper_state = b'\x00'
        self.pump_state = b'\x00'
        self.emergency = b'\x00'
        self.current_position = Float32MultiArray()

    def loop(self):
        while not rospy.is_shutdown():
            self.sendSerial()
            self.receiveSerial()
            self.rate.sleep()

    def sendSerial(self):
        uart_msg = struct.pack("<fffc??c", *self.move_cmd_theta, self.servo_angle, self.stepper_state, self.pump_state, self.emergency, b'\xFF')
        # rospy.loginfo(uart_msg)
        self.uart.write(uart_msg)

    def receiveSerial(self):
        # 受信と整形
        receiveData = self.uart.read(11)
        msg = struct.unpack("<ff?cc",receiveData)
        if(not(msg[3]==b'\x00' and msg[4]==b'\xff')):
            print(self.uart.readline())
            return;
        rospy.loginfo(msg)
        current_angle = Float32MultiArray(data=[msg[0], msg[1]])
        self.current_position.data = self.theta_to_cartesian(current_angle.data)
        self.theta_to_cartesian([0.5, 0.5])
        is_grabbed = Bool(data=msg[2])
        self.pub0.publish(current_angle)
        self.pub1.publish(is_grabbed)
        self.pub2.publish(self.current_position)

    def theta_to_cartesian(self, theta: List[float]):
        theta1 = math.pi * (theta[0] - 35) / 180
        theta2 = math.pi * (theta[1] - 138) / 180
        x = self.poi(self.l1 * math.cos(theta1) + self.l2*math.cos(theta1+theta2))
        y = self.poi(self.l1 * math.sin(theta1) + self.l2*math.sin(theta1+theta2))
        cartesian = list([x, y])
        return cartesian

    def cartesian_to_theta(self, cartesian: List[float]):
        x = cartesian[0]
        y = cartesian[1]
        theta1 = self.sign * math.acos(((x**2)+(y**2)+(self.l1**2)-(self.l2**2))/(2*self.l1*math.sqrt(x**2+y**2)))
        theta2 = math.atan((y-self.l1*math.sin(theta1))/(x-self.l1*math.cos(theta1)))-theta1
        theta1 = theta1 * 180 / math.pi + 35
        theta2 = theta2 * 180 / math.pi + 138
        theta = list([theta1, theta2])
        return theta

    def poi(self, arg: float):
        if math.fabs(arg) < 0.001:
            return 0
        else:
            return arg

    def rviz_simulator(self,msg):
        self.rviz_pub = rospy.Publisher("joint_states",JointState,queue_size=100)
        self.rviz_msg = JointState() 
        self.rviz_cmd = []
        
        self.rviz_msg.header = Header()
        self.rviz_msg.name = ['stand_arm1','arm1_arm2','arm2_linear','linear_wrist']
        self.rviz_msg.position = (self.move_cmd_theta[0],self.move_cmd_theta[1])
        self.rviz_pub.publish(self.rviz_msg)


if __name__ == "__main__":
    # try:
    rospy.init_node('device')
    rospy.loginfo("device : node is activated")
    device = device()
    # except:
    #     rospy.loginfo("device : something wrong")
    # finally:
    #     rospy.loginfo("device : process end")
