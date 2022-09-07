#!/usr/bin/env python3
import rospy
import math
from typing             import List
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header
from std_msgs.msg       import Float32MultiArray
from std_msgs.msg       import Bool
from std_msgs.msg       import Int8

class rviz_simulator():
    def __init__(self):
        rospy.loginfo("SCARA_rviz : constructor activated")
        self.pub_joint_states       = rospy.Publisher("joint_states", JointState, queue_size=100)
        self.pub_current_angle      = rospy.Publisher('current_angle', Float32MultiArray, queue_size=1)
        self.pub_current_position   = rospy.Publisher('current_position',Float32MultiArray,queue_size=1)

        self.sub_move_cmd           = rospy.Subscriber('move_cmd', Float32MultiArray, self.move_cmd_callback, queue_size=100)
        self.sub_servo_cmd          = rospy.Subscriber('servo_cmd', Bool, self.servo_angle_callback, queue_size=100)
        self.sub_color_field        = rospy.Subscriber('is_blue', Bool, self.is_blue_callback, queue_size=100)
        self.sub_current_state      = rospy.Subscriber('current_state',Int8,self.current_state_callback,queue_size = 100)
        
        self.rviz_msg = JointState()
        self.rviz_cmd = Float32MultiArray()
        self.rate = rospy.Rate(100)

        self.rviz_msg.header = Header()
        self.rviz_msg.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']       
        self.rviz_msg.position = [-2*math.pi/3, 2*math.pi/3, 0, 0]
        self.rviz_cmd_theta = [0,0]
        self.current_position = Float32MultiArray()
        self.l1 = 0.6
        self.l2 = 0.3
        self.sign = 1
        self.task_num = 1
        self.update()
    
    def is_blue_callback(self, msg):
        if self.task_num == 1:
            if msg.data == True:
                self.sign = 1
            else:
                self.sign = -1
       
    def servo_angle_callback(self, msg):
        self.servo_cmd = msg.data
        if msg.data == True:
            self.servo_angle = -(self.servo_angle[0] + self.servo_angle[1])
        elif msg.data == False:
            self.servo_angle = math.pi/2 - (self.servo_angle[0] + self.servo_angle[1])
        
        if self.servo_angle < 0:
            self.servo_angle = self.servo_angle + math.pi
        elif self.servo_angle > 2*math.pi:
            self.servo_angle = self.servo_angle - pi
            
        self.rviz_msg.position[2] = math.pi*self.servo_angle/180    
    
    def move_cmd_callback(self,msg):
        self.rviz_cmd.data = msg.data
        self.rviz_cmd_theta = self.cartesian_to_theta(self.rviz_cmd.data)
        self.rviz_msg.position = [self.rviz_cmd_theta[0],self.rviz_cmd_theta[1],0,0]
        
    def current_state_callback(self,msg):
        self.task_num = msg.data    

    def theta_to_cartesian(self, theta: List[float]):
        x = self.poi(self.l1 * math.cos(theta[0]) + self.l2*math.cos(theta[0]+theta[1]))
        y = self.poi(self.l1 * math.sin(theta[0]) + self.l2*math.sin(theta[0]+theta[1]))
        cartesian = list([x, y])
        return cartesian

    def cartesian_to_theta(self, cartesian: List[float]):
        x = cartesian[0]
        y = cartesian[1]
        theta1 = self.sign * math.acos(((x**2)+(y**2)+(self.l1**2)-(self.l2**2))/(2*self.l1*math.sqrt(x**2+y**2)))
        theta2 = math.atan((y-self.l1*math.sin(theta1))/(x-self.l1*math.cos(theta1)))-theta1
        theta = list([theta1, theta2])
        return theta
    
    def poi(self, arg: float):
        if math.fabs(arg) < 0.001:
            return 0
        else:
            return arg

    def update(self):
        rospy.loginfo("SCARA_rviz : enter main routine")
        while not rospy.is_shutdown():
            self.rviz_msg.header.stamp = rospy.Time.now()
            
            rospy.loginfo(self.rviz_msg.position)
            self.pub_joint_states.publish(self.rviz_msg)
            self.pub_current_position.publish(self.rviz_cmd)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("SCARA_rviz")
    func = rviz_simulator()
    rospy.loginfo("SCARA_rviz : end process")