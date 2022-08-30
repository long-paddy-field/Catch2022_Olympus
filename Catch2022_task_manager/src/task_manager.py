#!/usr/bin/env python3

from email.header import Header
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import rospy
import smach
import smach_ros


class Task1_Init(smach.State): #諸々の初期化待機
    def __init__(self):
        rospy.loginfo("task_manager : Task1_Init is activated")
        self.is_started = False
        self.jaguar_pos_x = [0,0,0,0,0,0,0,0]
        self.jaguar_pos_y = [0,0,0,0,0,0,0,0]
        self.box_pos_x = [0,0,0,0]
        self.box_pos_y = [0,0,0,0]
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("start_flag",Empty,self.start_flag_callback,queue_size = 1)
        rospy.Subscriber("is_blue",Bool,self.is_blue_callback,queue_size=1)
        self.r = rospy.Rate(30)
    
    def is_blue_callback(self,msg):
        if msg.data == True:
            self.jaguar_pos_x = [0,0,0,0,0,0,0,0]
            self.jaguar_pos_y = [0,0,0,0,0,0,0,0]
            self.box_pos_x = [0,0,0,0]
            self.box_pos_y = [0,0,0,0]
        elif msg.data == False:
            self.jaguar_pos_x = [0,0,0,0,0,0,0,0]
            self.jaguar_pos_y = [0,0,0,0,0,0,0,0]
            self.box_pos_x = [0,0,0,0]
            self.box_pos_y = [0,0,0,0]

        
    def start_flag_callback(self,msg):
        self.is_started = not self.is_started
        
    def execute(self, ud):
        while not rospy.is_shutdown():
            if self.is_started:
                rospy.loginfo("task_manager : Task1_Init is ended")
                return 'done'
            self.r.sleep()

class Task2_SeekWork(smach.State): #じゃがりこ探しに行く
    def __init__(self):
        smach.State.__init__(self,outcomes=['done','completed'])
        rospy.Subscriber("grab_cmd",Empty,self.grab_cmd_callback,queue_size = 1)
        self.target_pub = rospy.Publisher("target_location",JointState,queue_size = 1)
        self.end_flag = False
        self.task_counter = 0
        self.target_stand_arm1 = [0,0,0,0,0,0,0,0,0,0]
        self.target_arm1_arm2 = [0,0,0,0,0,0,0,0,0,0]
        self.target_arm2_linear = [0,0,0,0,0,0,0,0,0,0]
        self.target_linear_wrist = [0,0,0,0,0,0,0,0,0,0]
        self.r = rospy.Rate(30)
        
    def grab_cmd_callback(self,msg):
        self.end_flag = True
    
    def execute(self, ud):
        rospy.loginfo("task_manager : Task2_SeekWork is activated")
        if self.task_counter < 10 :   
            self.target_msg = JointState()
            self.target_msg.header = Header()
            self.target_msg.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']
            self.target_msg.position = [self.target_stand_arm1[self.task_counter],
                                        self.target_arm1_arm2[self.task_counter],
                                        self.target_arm2_linear[self.task_counter],
                                        self.target_linear_wrist[self.task_counter]]
            
            self.target_pub.publish(self.target_msg)
            while not rospy.is_shutdown():
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
        rospy.Subscriber("move_cmd",JointTrajectory,)
        self.task_counter = 0
        self.arm_hight = 0
        self.jaguar_hight = 90
        self.is_completed = False
        self.handstate = [0,0]
        self.r = rospy.Rate(30)
        
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
    
    def move_cmd_callback(self,msg):
        self.arm_hight = msg.points[2]
        
    def execute(self, ud):
        while not rospy.is_shutdown():
            if self.is_completed:
                self.task_counter = self.task_counter + 1
                if self.task_unter > 9:
                    self.task_counter = 0
                return 'done'
            self.r.sleep()

class Task4_SeekBox(smach.State):
    def __init__(self):
        rospy.loginfo("task_manager : Task4_SeekBox is activated")
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("release_cmd",Empty,queue_size = 1)
        self.target_pub = rospy.Publisher("target_location",JointState,queue_size = 1)

        self.is_released = False
        self.task_counter = 0
        self.target_stand_arm1 = [0,0,0,0,0,0,0,0,0,0]
        self.target_arm1_arm2 = [0,0,0,0,0,0,0,0,0,0]
        self.target_arm2_linear = [0,0,0,0,0,0,0,0,0,0]
        self.target_linear_wrist = [0,0,0,0,0,0,0,0,0,0]        
        self.r = rospy.Rate(30)
        
    def release_cmd_callback(self,msg):
        self.is_released = True
        
    def execute(self, ud):
        self.target_msg = JointState()
        self.target_msg.header = Header()
        self.target_msg.name = ['stand_arm1', 'arm1_arm2', 'arm2_linear', 'linear_wrist']
        self.target_msg.position = [self.target_stand_arm1[self.task_counter],
                                    self.target_arm1_arm2[self.task_counter],
                                    self.target_arm2_linear[self.task_counter],
                                    self.target_linear_wrist[self.task_counter]]
        
        self.target_pub.publish(self.target_msg)
        
        while not rospy.is_shutdown():
            if self.is_released:
                rospy.loginfo("task_manager : Task4_SeekBox is ended")
                self.is_released = False
                self.task_counter = self.task_counter + 1
                return 'done'
            self.r.sleep()
        
        
            
def main():
    rospy.init_node("task_manager")
     
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

# class State1(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['done', 'exit'])
#         self.counter = 0

#     def execute(self, userdata):
#         rospy.loginfo('Executing state STATE1')
#         rospy.sleep(2.0)
#         if self.counter < 3:
#             self.counter += 1
#             return 'done'
#         else:
#             return 'exit'


# class State2(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['done'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state STATE2')
#         rospy.sleep(2.0)
#         return 'done'


# class State3(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['done'])

#     def execute(self, userdata):
#         rospy.loginfo('Executing state STATE2')
#         rospy.sleep(2.0)
#         return 'done'

# # main


# def main():
#     rospy.init_node('smach_somple2')

#     sm_top = smach.StateMachine(outcomes=['succeeded'])
#     with sm_top:
#         smach.StateMachine.add('STATE3', State3(), transitions={'done': 'SUB'})

#         sm_sub = smach.StateMachine(outcomes=['done'])
#         with sm_sub:
#             smach.StateMachine.add('STATE1', State1(), transitions={
#                                    'done': 'STATE2', 'exit': 'done'})
#             smach.StateMachine.add(
#                 'STATE2', State2(), transitions={'done': 'STATE1'})

#         smach.StateMachine.add('SUB', sm_sub, transitions={
#                                'done': 'succeeded'})

#     sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
#     sis.start()
#     outcome = sm_top.execute()
#     rospy.spin()
#     sis.stop()


# if __name__ == '__main__':
#     main()
