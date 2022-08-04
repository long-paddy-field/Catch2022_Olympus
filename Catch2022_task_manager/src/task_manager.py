#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import Bool
from std_msgs.msg import Empty


class base_state(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['end_wait'])
        self.init_flag:bool = False
        self.loop_rate = 10
        
        rospy.Subscriber("hard_init", Empty,self.hardInitCallback, queue_size=1)
        
    def hardInitCallback(self,msg):
        self.init_flag = True
        
    def execute(self,ud):
        rospy.loginfo("waiting init cmd")
        self.r = rospy.Rate(self.loop_rate)
        
        while not rospy.is_shutdown():
            if init_flag:
                init_flag = False
                return 'end_wait'
            self.r.sleep()
            
            
class initialize(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['SuccessInit'])
        
    
    def execute(self, ud):
        rospy.loginfo("task_manager : Start setting...")
        while not rospy.is_shutdown():
            pass
             

class task_manager():
    def __init__(self):
        self.initializer()
        self.sm_top = smach.StateMachine(outcomes=["success"])
        with self.sm_base:
            smach.StateMachine.add("base_state",base_state(),transitions=)
        
    def initializer(self):
        rospy.init_node("task_manager")
        
        
        

if __name__=="__main__":
    try:
        rospy.loginfo("task_manager : node is activated")
        mytask = task_manager()
    except:
        rospy.loginfo("task_manager : something wrong")
    finally:
        rospy.loginfo("task_manager : process end")
    