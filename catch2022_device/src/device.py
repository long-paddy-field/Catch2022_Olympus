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
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Header
# from cobs import cobs
import serial
import struct
import math
import serial.tools.list_ports

port = serial.tools.list_ports.comports()[0].device
# port="/dev/pts/4"
mode = "real"


class device():

    def __init__(self):
        self.setup()
        self.loop()

    def move_rad_callback(self, msg):
        self.move_deg[0] = msg.data[0]*180/math.pi+125
        self.move_deg[1] = msg.data[1]*180/math.pi+138
        # rospy.loginfo(uart_msg)
        # motor=struct.pack('<ff',*move_cmd)
        # self.uart.write(motor)
        # rospy.loginfo(motor)

    def servo_angle_callback(self, msg):
        self.servo_angle = msg.data*180/math.pi

    def stepper_state_callback(self, msg):
        self.stepper_state = msg.data.to_bytes(1, 'little')

    def pmp_state_callback(self, msg):
        self.pmp_state = msg.data.to_bytes(1, 'little')

    def emergency_callback(self, msg):
        self.emergency = msg.data

    def led_hsv_callback(self, msg):
        self.led_hsv = msg.data

    def connect_device_callback(self, msg):
        self.uart.reset_input_buffer()
        rospy.loginfo("device: connect_decvice received")
        if not self.connect_flag:
            self.uart.write(b'\xFF\xFF\xFF\xFF')
            receive = self.uart.read(4)
            if receive == b'\xFF\xFF\xFF\xFF':
                self.connect_flag = True
                self.pub_is_connected.publish()
                rospy.loginfo("is_connected published")

    def device_start_callback(self, msg):
        self.uart.reset_input_buffer()
        rospy.loginfo("device: device_start received")
        if not self.start_flag:
            self.uart.write(b'\xFF\xFF\xFF\xFF')
            receive = self.uart.read(4)
            if receive == b'\xFF\xFF\xFF\xFF':
                self.start_flag = True

    def setup(self):
        global port
        global mode
        self.connect_flag = False
        self.start_flag = False
        self.uart = serial.Serial(port, 115200)

        self.pub_current_angle = rospy.Publisher('current_angle', Float32MultiArray, queue_size=100)
        self.pub_is_grabbed = rospy.Publisher('is_grabbed', Int8, queue_size=100)
        self.pub_is_connected = rospy.Publisher("is_connected", Empty, queue_size=100)

        self.rate = rospy.Rate(100)

        # subscriberの宣言
        self.sub_move_rad = rospy.Subscriber('move_rad', Float32MultiArray, self.move_rad_callback, queue_size=100)
        self.sub_servo_angle = rospy.Subscriber('servo_angle', Float32, self.servo_angle_callback, queue_size=100)
        self.sub_stepper_state = rospy.Subscriber('stepper_state', Int8, self.stepper_state_callback, queue_size=100)
        self.sub_pmp_state = rospy.Subscriber('pmp_state', Int8, self.pmp_state_callback, queue_size=100)
        self.sub_emergency = rospy.Subscriber('emergency', Int8, self.emergency_callback, queue_size=100)
        self.sub_led_hsv = rospy.Subscriber('led_hsv', Int16MultiArray, self.led_hsv_callback, queue_size=100)
        self.sub_connect_device = rospy.Subscriber('connect_device', Empty, self.connect_device_callback, queue_size=100)
        self.sub_device_start = rospy.Subscriber('device_start', Empty, self.device_start_callback, queue_size=100)

        # 変数の初期化
        self.move_deg = [155, 22]
        self.past_deg = [0, 0]
        self.move_cmd_theta = [90, 78]
        self.servo_angle = 0x00
        self.stepper_state = b'\x00'
        self.pmp_state = b'\x00'
        self.emergency = b'\x00'
        self.led_hsv = [0, 0, 0]


        while not self.connect_flag:
            pass
        rospy.loginfo("Connection Established")
            
        while not self.start_flag:
            pass
        rospy.loginfo("Start")

    def loop(self):
        while not rospy.is_shutdown():
            self.sendSerial()
            self.receiveSerial()
            # # if mode == "sim":
            #     self.current_angle = self.move_cmd_theta
            #     rospy.loginfo(self.current_angle)
            # self.rviz_msg.header.stamp = rospy.Time.now()
            # self.rviz_simulator()
            self.rate.sleep()

    def sendSerial(self):
        uart_msg = struct.pack("<fffcc?hccc", *self.move_deg, self.servo_angle, self.stepper_state, self.pmp_state, self.emergency,
                               self.led_hsv[0], self.led_hsv[1].to_bytes(1, 'little'), self.led_hsv[2].to_bytes(1, 'little'), b'\xFF')
        # uart_msg = struct.pack("<fffcc?c", *self.move_deg, self.servo_angle, self.stepper_state, self.pmp_state, self.emergency,b'\xFF')
        # rospy.loginfo(uart_msg)
        self.uart.write(uart_msg)

    def receiveSerial(self):
        # 受信と整形
        receiveData = self.uart.read(11)
        msg = struct.unpack("<ffccc", receiveData)
        if (not (msg[3] == b'\x00' and msg[4] == b'\xff')):
            print(self.uart.readline())
            return
        rospy.loginfo(msg)
        self.current_angle = Float32MultiArray(data=[(msg[0]-125)*math.pi/180, (msg[1]-138)*math.pi/180])
        is_grabbed = Int8(data=int.from_bytes(msg[2], 'little'))
        self.pub_current_angle.publish(self.current_angle)
        self.pub_is_grabbed.publish(is_grabbed)


if __name__ == "__main__":

    # try:
    rospy.init_node('device')
    rospy.loginfo("device : node is activated")
    # mode = rospy.get_param("mode")
    device = device()
    # except:
    #     rospy.loginfo("device : something wrong")
    # finally:
    #     rospy.loginfo("device : process end")
