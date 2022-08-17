#!/usr/bin/env python3

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose2D
import rospy
import smach
import smach_ros

class Task1_Init(smach.State): #諸々の初期化待機
    def __init__(self):
        rospy.loginfo("task_manager : Task1_Init activated")
        self.is_started = False
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("start_flag",Empty,self.start_flag_callback,queue_size = 1)
        
    def start_flag_callback(self):
        self.is_started = not self.is_started
        
    def execute(self, ud):
        while not rospy.is_shutdown():
            if self.is_started:
                rospy.loginfo("task_manager : Task1_Init ended")
                return 'done'

class Task2_SeekWork(smach.State): #じゃがりこ探しに行く
    def __init__(self):
        rospy.loginfo("task_manager : Task2_SeekWork activated")
        smach.State.__init__(self,outcomes=['done'])
        rospy.Subscriber("grab_cmd",Empty,self.grab_cmd_callback,queue_size = 1)
        self.target_pub = rospy.Publisher("target_location",Pose2D,queue_size = 1)
        self.end_flag = False
        
    def grab_cmd_callback(self):
        self.end_flag = True
    
    def execute(self, ud):
        
        if self.end_flag:
            rospy.loginfo("task_manager : Task2_SeekWork ended")
            self.end_flag = False
            return 'done'
        
def main():
    rospy.init_node("task_manager")
     
    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        pass    

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
