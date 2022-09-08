#!/usr/bin/env python3

from email.header import Header
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
import rospy
import smach
import smach_ros
import math

jaguar_pos_x = [0,0,0,0,0,0,0,0,0,0]
jaguar_pos_y = [0,0,0,0,0,0,0,0,0,0]
box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
box_pos_y    = [0,0,0,0,0,0,0,0,0,0]
is_Handy     = False
task_num     = Int8(data = 1)

def is_handy_callback(msg):
    global is_Handy
    
    is_Handy = msg.data

def cal_dist(x0:float,y0:float,x1:float,y1:float):
    return math.sqrt((x1-x0)**2+(y1-y0)**2)

def jaguar_position_callback(msg):
    global jaguar_pos_x
    global jaguar_pos_y
    
    tag = 0
    min_dist = 100
    
    if not len(msg.data) == 0 or (msg.data[0] == 0 and msg.data[1] == 0):
        for i in range(10):
            dist = cal_dist(msg.data[0],msg.data[1],jaguar_pos_x[i],jaguar_pos_y[i])
            if dist < min_dist:
                min_dist = dist
                tag = i
        
        jaguar_pos_x[tag] = msg.data[0]
        jaguar_pos_y[tag] = msg.data[1]        

def pub_current_state():
    global task_num
    pub_state = rospy.Publisher("current_state",Int8,queue_size=100)
    pub_state.publish(task_num)
    

class Task1_Init(smach.State): #諸々の初期化待機
    def __init__(self):
        rospy.loginfo("task_manager : Task1_Init is activated")
        self.is_started = False
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("start_flag",Empty,self.start_flag_callback,queue_size = 100)
        rospy.Subscriber("is_blue",Bool,self.is_blue_callback,queue_size = 100)
        self.r = rospy.Rate(100)
    
    def is_blue_callback(self,msg):
        global jaguar_pos_x
        global jaguar_pos_y
        global box_pos_x
        global box_pos_y
        if msg.data == True:
            rospy.loginfo("task_manager : blue side")
            jaguar_pos_x = [0,0,0,0,0,0,0,0,0,0]
            jaguar_pos_y = [0,0,0,0,0,0,0,0,0,0]
            box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
            box_pos_y    = [0,0,0,0,0,0,0,0,0,0]
        elif msg.data == False:
            rospy.loginfo("task_manager : red side")
            jaguar_pos_x = [0,0,0,0,0,0,0,0,0,0]
            jaguar_pos_y = [0,0,0,0,0,0,0,0,0,0]
            box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
            box_pos_y    = [0,0,0,0,0,0,0,0,0,0]

    def start_flag_callback(self,msg):
        self.is_started = not self.is_started
        
    def execute(self, ud):
        global task_num
        task_num = 1
        pub_current_state()
        while not rospy.is_shutdown():
            if self.is_started:
                rospy.loginfo("task_manager : Task1_Init is ended")
                return 'done'
            self.r.sleep()

class Task2_SeekWork(smach.State): #じゃがりこ探しに行く
    def __init__(self):
        smach.State.__init__(self,outcomes=['done','completed'])
        rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size = 100)
        rospy.Subscriber("grab_cmd",Empty,self.grab_cmd_callback,queue_size = 100)
        self.pub_servo_cmd  = rospy.Publisher("servo_cmd",Bool,queue_size=100)
        self.target_pub     = rospy.Publisher("target_location",Float32MultiArray,queue_size = 100)
        self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=100)
        self.stepper_state  = Int8(data = 0)
        self.end_flag       = False
        self.task_counter   = 0
        self.servo_cmd      = Bool(data = False)
        self.r = rospy.Rate(30)
        
    def end_cmd_callback(self,msg):
        rospy.loginfo("nya")
        self.end_flag = True
    
    def grab_cmd_callback(self,msg):
        self.end_flag = True
    
    def execute(self, ud):
        global jaguar_pos_x
        global jaguar_pos_y
        global box_pos_x
        global box_pos_y        
        rospy.loginfo("task_manager : Task2_SeekWork is activated")
        if self.task_counter < 10 : 
            if is_Handy == False:  
                self.target_msg = Float32MultiArray()
                self.target_msg.data = [jaguar_pos_x[self.task_counter],jaguar_pos_y[self.task_counter]]
                self.target_pub.publish(self.target_msg)

            if self.task_counter == 1 or self.task_counter == 2:
                self.stepper_state.data = 3
            else:
                self.stepper_state.data = 1
            self.stepper_state_pub.publish(self.stepper_state)
                
            while not rospy.is_shutdown():
                if not is_Handy:
                    if self.task_counter == 1 or self.task_counter == 2:
                        self.servo_cmd = True
                    else : 
                        self.servo_cmd = False
                    self.pub_servo_cmd.publish(self.servo_cmd)    
                if self.end_flag:
                    rospy.loginfo("task_manager : Task2_SeekWork is ended")
                    self.task_counter = self.task_counter + 1
                    self.end_flag = False
                    return 'done'
                self.r.sleep()
        else :
            rospy.loginfo("3nya")
            rospy.loginfo("task_manager : all tasks completed")
            self.task_counter = 0
            return 'completed'
        
class Task3_GrabWork(smach.State):
    def __init__(self):
        rospy.loginfo("task_manager : Task3_GrabWork is activated")
        smach.State.__init__(self, outcomes=['done'])
        rospy.Subscriber("is_grabbed",Int8MultiArray,self.is_grabbed_callback,queue_size=1)
        rospy.Subscriber("step_cmd",Bool,self.step_cmd_callback,queue_size=1)
        self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=1)
        self.stepper_state = Int8(data = 0)
        self.task_counter   = 0
        self.arm_hight      = 0
        self.jaguar_hight   = 90
        self.is_completed   = False
        self.handstate      = [0,0]
        self.r              = rospy.Rate(30)
        self.loop_counter   = 0

    def step_cmd_callback(self,msg):
        if is_Handy == True:
            if msg.data == True:#上昇
                if self.stepper_state == 3:
                    self.stepper_state = 0
                else:
                    self.stepper_state = self.stepper_state - 1
        #     if self.stepper_state == 2:
        #         self.stepper_state = 1
        #     elif self.stepper_state == 4:
        #         self.stepper_state.data = 3
        #     elif self.stepper_state == 1 or self.stepper_state == 3:
        #         self.stepper_state.data = 0  

            elif msg.data == False:#下降
                if self.stepper_state == 0 and (self.task_counter == 1 or self.task_counter ==2):
                # if self.task_counter == 1 or self.task_counter == 2:
                    self.stepper_state = 3
                # else:
                #     self.stepper_state = 1
                else:
                    self.stepper_state = self.stepper_state + 1
            # elif self.stepper_state == 1:
            #     self.stepper_state = 2
            # elif self.stepper_state == 3:
            #     self.stepper_state = 4
            self.stepper_state_pub.publish(self.stepper_state)

    def is_grabbed_callback(self,msg):
        self.handstate = msg.data
        if self.task_counter < 7:
            if msg.data[0] == 1 and msg.data[1] == 1 and self.arm_hight > self.jaguar_hight:
                self.is_completed = True
            else :
                self.is_completed = False
        else:
            if (msg.data[0] == 1 or msg.data[1] == 1) and self.arm_hight > self.jaguar_hight:
                self.is_completed = True
            else :
                self.is_completed = False
        
    def execute(self, ud):
        while not rospy.is_shutdown():            
            if self.loop_counter == 0:
                if self.task_counter == 1 or self.task_counter == 2:
                    self.stepper_state.data = 4
                else:
                    self.stepper_state.data = 2
            elif self.loop_counter == 60:
                 self.stepper_state.data = 0
                 self.loop_counter = 0

            self.stepper_state_pub.publish(self.stepper_state)

            if self.is_completed and self.loop_counter > 70:
                self.task_counter = self.task_counter + 1
                self.loop_counter = 0
                if self.task_counter > 9:
                    self.task_counter = 0
                return 'done'
            elif not self.is_completed and self.loop_counter > 70:
                self.loop_counter = 0
            self.loop_counter = self.loop_counter + 1
            self.r.sleep()

class Task4_SeekBox(smach.State):
    def __init__(self):
        rospy.loginfo("task_manager : Task4_SeekBox is activated")
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("release_cmd",Empty,queue_size = 1)
        rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size=1)
        rospy.Subscriber("step_cmd",Bool,self.step_cmd_callback,queue_size=1)
        self.pub_servo_cmd  = rospy.Publisher("servo_cmd",Bool,queue_size=100)
        self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=1)
        self.stepper_state = Int8(data = 0)
        self.target_pub = rospy.Publisher("target_location",Float32MultiArray,queue_size = 1)

        self.is_released = False
        self.task_counter = 0      
        self.r = rospy.Rate(30)
        self.servo_cmd = Bool(data = False)
        
    def release_cmd_callback(self,msg):
        self.is_released = True
    
    def end_cmd_callback(self,msg):
        self.stepper_state.data == 2
        self.stepper_state_pub.publish(self.stepper_state)
    
    
    def step_cmd_callback(self,msg):
        if is_Handy == True:
            if msg.data == True:#上昇
                if self.stepper_state == 3:
                    self.stepper_state = 0
                else:
                    self.stepper_state = self.stepper_state - 1
        #     if self.stepper_state == 2:
        #         self.stepper_state = 1
        #     elif self.stepper_state == 4:
        #         self.stepper_state.data = 3
        #     elif self.stepper_state == 1 or self.stepper_state == 3:
        #         self.stepper_state.data = 0  

            elif msg.data == False:#下降
                if self.stepper_state == 0 and (self.task_counter == 1 or self.task_counter ==2):
                # if self.task_counter == 1 or self.task_counter == 2:
                    self.stepper_state = 3
                # else:
                #     self.stepper_state = 1
                else:
                    self.stepper_state = self.stepper_state + 1
            # elif self.stepper_state == 1:
            #     self.stepper_state = 2
            # elif self.stepper_state == 3:
            #     self.stepper_state = 4
            self.stepper_state_pub.publish(self.stepper_state)
        
        
    def execute(self, ud):
        global jaguar_pos_x
        global jaguar_pos_y
        global box_pos_x
        global box_pos_y
        global is_Handy
        
        if is_Handy == False:
            self.target_msg = Float32MultiArray()
            self.target_msg.data = [box_pos_x[self.task_counter],
                                        box_pos_y[self.task_counter]]
                                    
        self.target_pub.publish(self.target_msg)
        
        while not rospy.is_shutdown():
            if not is_Handy:
                if self.task_counter == 6 or self.task_counter == 7:
                    self.servo_cmd = True
                else:
                    self.servo_cmd = False
                self.pub_servo_cmd.publish(self.servo_cmd)
            if self.is_released:
                rospy.loginfo("task_manager : Task4_SeekBox is ended")
                self.is_released = False
                self.task_counter = self.task_counter + 1
                return 'done'
            self.r.sleep()
                    
def main():
    rospy.init_node("task_manager")
     
    rospy.Subscriber('is_handy',Bool,is_handy_callback,queue_size = 100)
    rospy.Subscriber('jaguar_position',Float32MultiArray,jaguar_position_callback,queue_size = 100)
    
    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('Init',Task1_Init(),transitions={'done':'SeekWork'})
        smach.StateMachine.add('SeekWork',Task2_SeekWork(),transitions={'done':'GrabWork','completed':'succeeded'})
        smach.StateMachine.add('GrabWork',Task3_GrabWork(),transitions={'done':'SeekBox'})
        smach.StateMachine.add('SeekBox',Task4_SeekBox(),transitions={'done':'SeekWork'})
        
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()