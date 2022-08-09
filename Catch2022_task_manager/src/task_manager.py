#!/usr/bin/env python3

import rospy
import smach
import smach_ros


class State1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'exit'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE1')
        rospy.sleep(2.0)
        if self.counter < 3:
            self.counter += 1
            return 'done'
        else:
            return 'exit'


class State2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE2')
        rospy.sleep(2.0)
        return 'done'


class State3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, userdata):
        rospy.loginfo('Executing state STATE2')
        rospy.sleep(2.0)
        return 'done'

# main


def main():
    rospy.init_node('smach_somple2')

    sm_top = smach.StateMachine(outcomes=['succeeded'])
    with sm_top:
        smach.StateMachine.add('STATE3', State3(), transitions={'done': 'SUB'})

        sm_sub = smach.StateMachine(outcomes=['done'])
        with sm_sub:
            smach.StateMachine.add('STATE1', State1(), transitions={
                                   'done': 'STATE2', 'exit': 'done'})
            smach.StateMachine.add(
                'STATE2', State2(), transitions={'done': 'STATE1'})

        smach.StateMachine.add('SUB', sm_sub, transitions={
                               'done': 'succeeded'})

    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    
# #!/usr/bin/env python3

# import rospy
# import smach
# from std_msgs.msg import Bool
# from std_msgs.msg import Empty


# class base_state(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,outcomes=['end_wait'])
#         self.init_flag:bool = False
#         self.loop_rate = 10
        
#         rospy.Subscriber("hard_init", Empty,self.hardInitCallback, queue_size=1)
        
#     def hardInitCallback(self,msg):
#         self.init_flag = True
        
#     def execute(self,ud):
#         rospy.loginfo("waiting init cmd")
#         self.r = rospy.Rate(self.loop_rate)
        
#         while not rospy.is_shutdown():
#             if init_flag:
#                 init_flag = False
#                 return 'end_wait'
#             self.r.sleep()
            
            
# class initialize(smach.State):
#     def __init__(self):
#         smach.State.__init__(self,outcomes=['SuccessInit'])
        
    
#     def execute(self, ud):
#         rospy.loginfo("task_manager : Start setting...")
#         while not rospy.is_shutdown():
#             pass
             

# class task_manager():
#     def __init__(self):
#         self.initializer()
#         self.sm_top = smach.StateMachine(outcomes=["success"])
#         with self.sm_base:
#             smach.StateMachine.add("base_state",base_state(),transitions=)
        
#     def initializer(self):
#         rospy.init_node("task_manager")
        
        
        

# if __name__=="__main__":
#     try:
#         rospy.loginfo("task_manager : node is activated")
#         mytask = task_manager()
#     except:
#         rospy.loginfo("task_manager : something wrong")
#     finally:
#         rospy.loginfo("task_manager : process end")
    