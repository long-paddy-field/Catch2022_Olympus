#!/usr/bin/env python3

from faulthandler import is_enabled
import queue
import rospy
import smach
import smach_ros
import math
from playsound import playsound
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int8
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray



own_arm_pos_x = [0, 0, 0, 0, 0, 0, 0, 0]
own_arm_pos_y = [0, 0, 0, 0, 0, 0, 0, 0]
com_arm_pos_x = [0, 0, 0, 0, 0, 0, 0, 0, 0]
com_arm_pos_y = [0, 0, 0, 0, 0, 0, 0, 0, 0]
box_pos_x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
box_pos_y = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

own_jaguar_pos_x = [0.535, 0.435, 0.435, 0.335, 0.235, 0.235, 0.135, 0.035, 0.035, -0.065, -0.165, -0.165, -0.265, -0.365, -0.365, -0.465]
own_jaguar_pos_y = [-0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448]
com_jaguar_pos_x = [0.595, 0.455, 0.315, 0.175, 0.035, -0.105, -0.245, -0.385, -0.525]
com_jaguar_pos_y = [-0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845]
box_jaguar_pos_x = [0.585, 0.685, 0.785, 0.585, 0.685, 0.785, 0.585, 0.685, 0.785, 0.585, 0.685, 0.785,0.585, 0.685, 0.785, 0.585, 0.685, 0.785]
box_jaguar_pos_y = [0.035, 0.035, 0.035, 0.135, 0.135, 0.135, 0.236, 0.236, 0.236, 0.336, 0.336, 0.336, 0.437, 0.437, 0.437, 0.537, 0.537, 0.537]


def cal_dist(x0:float,y0:float,x1:float,y1:float):
    return math.sqrt((x1-x0)**2+(y1-y0)**2)

def jaguar_position_callback(msg):
    global own_arm_pos_x
    global own_arm_pos_y
    global own_jaguar_pos_x
    global own_jaguar_pos_y
    global com_jaguar_pos_x
    global com_jaguar_pos_y
    global box_jaguar_pos_x
    global box_jaguar_pos_y

    tag = 0
    min_dist = 100

    if not len(msg.data) == 0 or (msg.data[0] == 0 and msg.data[1] == 0):
        for i in range(10):
            dist = cal_dist(msg.data[0],msg.data[1],own_arm_pos_x[i],own_arm_pos_y[i])
            if dist < min_dist:
                min_dist = dist
                tag = i
        own_arm_pos_x[tag] = msg.data[0]
        own_arm_pos_y[tag] = msg.data[1]

class Init(smach.State): #諸々の初期化待機
    def __init__(self):
        self.field_color = rospy.get_param("~field_color")
        rospy.loginfo(self.field_color)
        self.is_Handy = True

        self.is_started = False
        self.auto_flag  = False
        smach.State.__init__(self,outcomes=['done'])

        self.pub_target_location     = rospy.Publisher("target_location",Float32MultiArray,queue_size = 100)
        self.pub_servo_cmd           = rospy.Publisher("servo_cmd", Int8, queue_size=100)
        self.pub_led_hsv             = rospy.Publisher("led_hsb",Int16MultiArray,queue_size=100)
        self.target_location = Float32MultiArray()
        self.servo_cmd = Int8(data=0)
        self.led_hsv = Int16MultiArray(data=[0,255,255])
        self.past_connect = False

        if self.field_color == "blue":
            self.target_location.data = [0.3 * math.sqrt(3),0]
        elif self.field_color == "red":
            self.target_location.data = [0.3 * math.sqrt(3),0]

        self.r = rospy.Rate(10)

    def start_cmd_callback(self,msg):
        self.is_started = not self.is_started

    def is_handy_callback(self,msg):
        self.past_is_handy = self.is_Handy
        self.is_Handy = msg.data
        if self.is_Handy:
            self.auto_flag = False
    def end_cmd_callback(self,msg):
        pass
    
    def is_connected_callback(self,msg):
        if not self.past_connect and msg.data:
            playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/CONNECT.wav")
        
        self.past_connect = msg.data

    def execute(self, ud):
        rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size = 100)
        rospy.Subscriber("is_handy",Bool,self.is_handy_callback,queue_size=100)
        rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size=100)
        rospy.Subscriber("is_connected",Bool,self.is_connected_callback,queue_size=100)
        self.is_started = False
        self.auto_flag = False
        
        self.led_hsv.data = [240,255,255]
        self.pub_led_hsv.publish(self.led_hsv)

        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/START.wav")
        
        while not rospy.is_shutdown():
            if not self.is_Handy and not self.auto_flag:
                self.pub_target_location.publish(self.target_location)
                self.led_hsv.data = [0,255,255]
                self.pub_led_hsv.publish(self.led_hsv)

                self.auto_flag = True
            if not self.is_Handy:
                self.pub_servo_cmd.publish(self.servo_cmd)
            if self.is_started:
                rospy.loginfo("task_manager : Task1_Init is ended")
                return 'done'
            self.r.sleep()
            
class SeekOwn(smach.State):
    def __init__(self):
        global own_arm_pos_x
        global own_arm_pos_y
        
        smach.State.__init__(self,outcomes=['done','completed'])
        
        self.pub_target_location = rospy.Publisher("target_location",Float32MultiArray,queue_size=100)
        self.pub_servo_cmd       = rospy.Publisher("servo_cmd",Int8,queue_size=100)
        self.pub_stepper_state   = rospy.Publisher("stepper_state",Int8,queue_size=100)
        self.pub_pmp_state       = rospy.Publisher("pmp_state",Int8,queue_size=100)
        
        self.target_location = Float32MultiArray(data=[0,0])
        self.servo_cmd = Int8(data = 0)
        self.stepper_state = Int8(data = 1)
        self.pmp_state = Int8(data = 0)
        
        self.task_counter = 0
        self.target_x = own_arm_pos_x
        self.target_y = own_arm_pos_y
        
        self.start_cmd = False
        self.is_handy = True
        self.is_enable = True
        self.is_ended = True
        
        self.r = rospy.Rate(10)
    
    def start_cmd_callback(self,msg):
        self.start_cmd = True
    
    def end_cmd_callback(self,msg):
        self.is_ended = True
        
    def is_handy_callback(self,msg):
        self.past_is_handy = self.is_handy
        self.is_handy = msg.data
        if self.past_is_handy and not self.is_handy:
            self.is_enable = True
            self.is_ended = False
            playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/AUTO.wav")
        elif not self.past_is_handy and self.is_handy:
            playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/Hand.wav")
            
    
    def execute(self, ud):
        rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size=100)
        rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size=100)
        rospy.Subscriber("is_handy",Bool,self.is_handy_callback,queue_size=100)

        self.start_cmd = False
        self.is_enable = True
        self.is_ended = False
        self.target_location.data = [self.target_x[self.task_counter],self.target_y[self.task_counter]]
        
        if self.task_counter >= 8:
            rospy.loginfo('all task completed')
            return 'completed'
        
        while not rospy.is_shutdown():
            self.pub_stepper_state.publish(self.stepper_state)
            self.pub_pmp_state.publish(self.pmp_state)
            if not self.is_handy:
                if self.task_counter % 3 == 1:
                    self.servo_cmd = 1
                else:
                    self.servo_cmd = 0
                self.pub_servo_cmd.publish(self.servo_cmd)
                if self.is_enable and not self.is_ended:
                    rospy.loginfo(self.target_location.data)
                    self.pub_target_location.publish(self.target_location)
                    self.is_enable = False
                    self.is_ended  = False
            if self.start_cmd:
                rospy.loginfo("SeekOwn end")
                self.task_counter += 1
                return 'done'
            self.r.sleep()
    
class GrabOwn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.pub_pmp_state = rospy.Publisher("pmp_state",Int8,queue_size=100)
        self.pub_stepper_state = rospy.Publisher("stepper_state",Int8,queue_size=100)
        
        self.is_completed = False
        self.stepper_state = Int8(data=2)
        self.pmp_state = Int8(data=3)
        self.r = rospy.Rate(0.5)
        
    def is_grabbed_callback(self,msg):
        if msg.data == 3:
            self.is_completed = True
        else:
            self.is_completed = False
        
    def start_cmd_callback(self,msg):
        self.is_completed = True
        
    def execute(self, ud):
        rospy.Subscriber("is_grabbed",Int8,self.is_grabbed_callback,queue_size=100)
        rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size=100)
        self.is_completed = False
        self.pub_pmp_state.publish(self.pmp_state)
        
        while not rospy.is_shutdown():
            self.stepper_state.data = 2
            self.pub_stepper_state.publish(self.stepper_state)
            self.r.sleep()
            self.stepper_state.data = 0
            self.pub_stepper_state.publish(self.stepper_state)
            self.r.sleep()
            if self.is_completed == True:
                return 'done'

class SeekCom(smach.State):
    def __init__(self):
        global com_arm_pos_x
        global com_arm_pos_y

        smach.State.__init__(self, outcomes=['done','completed'])

        self.pub_target_location = rospy.Publisher("target_location", Float32MultiArray, queue_size=100)
        self.pub_servo_cmd = rospy.Publisher("servo_cmd", Int8, queue_size=100)
        self.pub_stepper_state = rospy.Publisher("stepper_state", Int8, queue_size=100)
        self.pub_pmp_state = rospy.Publisher("pmp_state",Int8,queue_size=100)

        self.target_location = Float32MultiArray(data=[0, 0])
        self.servo_cmd = Int8(data=0)
        self.stepper_state = Int8(data=3)
        self.pmp_state = Int8(data=0)

        self.task_counter = 0
        self.target_x = com_arm_pos_x
        self.target_y = com_arm_pos_y

        self.start_cmd = False
        self.is_handy = True
        self.is_enable = True
        self.is_ended = True

        self.r = rospy.Rate(10)

    def start_cmd_callback(self, msg):
        self.start_cmd = True

    def end_cmd_callback(self, msg):
        self.is_ended = True

    def is_handy_callback(self,msg):
        self.past_is_handy = self.is_handy
        self.is_handy = msg.data
        if self.past_is_handy and not self.is_handy:
            self.is_enable = True
            self.is_ended = False

    def execute(self, ud):
        rospy.Subscriber("start_cmd", Empty, self.start_cmd_callback, queue_size=100)
        rospy.Subscriber("end_cmd", Empty, self.end_cmd_callback, queue_size=100)
        rospy.Subscriber("is_handy", Bool, self.is_handy_callback, queue_size=100)

        self.start_cmd = False
        self.is_enable = True
        self.is_ended = False
        self.target_location.data = [self.target_x[self.task_counter], self.target_y[self.task_counter]]

        if self.task_counter >= 9:
            rospy.loginfo('all task completed')
            return 'completed'

        while not rospy.is_shutdown():
            self.pub_stepper_state.publish(self.stepper_state)
            if not self.is_handy:
                if self.task_counter % 2 == 1:
                    self.servo_cmd = 2
                    self.pmp_state.data = 1
                else:
                    self.servo_cmd = 0
                    self.pmp_state.data = 0
                self.pub_pmp_state.publish(self.pmp_state)
                self.pub_servo_cmd.publish(self.servo_cmd)
                if self.is_enable and not self.is_ended:
                    self.pub_target_location.publish(self.target_location)
                    self.is_enable = False
                    self.is_ended = False
            if self.start_cmd:
                rospy.loginfo("SeekOwn end")
                self.task_counter += 1
                return 'done'
            self.r.sleep()
            
class GrabCom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','second'])

        self.pub_pmp_state = rospy.Publisher("pmp_state", Int8, queue_size=100)
        self.pub_stepper_state = rospy.Publisher("stepper_state", Int8, queue_size=100)

        self.task_counter = 0
        self.is_completed = False
        self.stepper_state = Int8(data=4)
        self.pmp_state = Int8(data=3)
        self.r = rospy.Rate(0.5)

    def is_grabbed_callback(self,msg):
        if msg.data == 3 and self.task_counter % 2 == 1:
            self.is_completed = True
        elif msg.data == 1 and self.task_counter % 2 == 0:
            self.is_completed = True
        else:
            self.is_completed = False
    
    def start_cmd_callback(self,msg):
        self.is_completed = True
        
    def execute(self, ud):
        rospy.Subscriber("is_grabbed",Int8,self.is_grabbed_callback,queue_size=100)
        rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size=100)
        self.is_completed = False
        self.pub_pmp_state.publish(self.pmp_state)
        
        while not rospy.is_shutdown():
            self.stepper_state.data = 4
            self.pub_stepper_state.publish(self.stepper_state)
            self.r.sleep()
            self.stepper_state.data = 0
            self.pub_stepper_state.publish(self.stepper_state)
            self.r.sleep()
            if self.is_completed == True:
                if self.task_counter % 2 == 0 and not self.task_counter == 8:                
                    self.task_counter += 1
                    return 'second'
                else:
                    self.task_counter += 1
                    return 'done'
                    
class SeekBox(smach.State):
    def __init__(self):
        global box_pos_x
        global box_pos_y

        smach.State.__init__(self, outcomes=['done'])

        self.pub_target_location = rospy.Publisher("target_location", Float32MultiArray, queue_size=100)
        self.pub_servo_cmd = rospy.Publisher("servo_cmd", Int8, queue_size=100)
        self.pub_stepper_state = rospy.Publisher("stepper_state", Int8, queue_size=100)
        self.pub_pmp_state = rospy.Publisher("pmp_state",Int8,queue_size=100)

        self.target_location = Float32MultiArray(data=[0, 0])
        self.servo_cmd = Int8(data=0)
        self.stepper_state = Int8(data=0)
        self.pmp_state = Int8(data = 3)

        self.task_counter = 0
        self.target_x = box_pos_x
        self.target_y = box_pos_y

        self.start_cmd = False
        self.is_handy = True
        self.is_enable = True
        self.is_ended = True

        self.r = rospy.Rate(10)

    def start_cmd_callback(self, msg):
        self.start_cmd = True

    def end_cmd_callback(self, msg):
        self.is_ended = True

    def is_handy_callback(self, msg):
        self.past_is_handy = self.is_handy
        self.is_handy = msg.data
        if self.past_is_handy and not self.is_handy:
            self.is_enable = True
            self.is_ended = False

    def execute(self, ud):
        rospy.Subscriber("start_cmd", Empty, self.start_cmd_callback, queue_size=100)
        rospy.Subscriber("end_cmd", Empty, self.end_cmd_callback, queue_size=100)
        rospy.Subscriber("is_handy", Bool, self.is_handy_callback, queue_size=100)

        self.start_cmd = False
        self.is_enable = True
        self.is_ended = False
        self.target_location.data = [self.target_x[self.task_counter], self.target_y[self.task_counter]]

        while not rospy.is_shutdown():
            self.pub_stepper_state.publish(self.stepper_state)
            self.pub_pmp_state.publish(self.pmp_state)
            if not self.is_handy:
                if self.task_counter % 4 == 1:
                    self.servo_cmd = 0
                elif  self.task_counter % 4 == 2:
                    self.servo_cmd = 2
                else:
                    self.servo_cmd = 1
                self.pub_servo_cmd.publish(self.servo_cmd)
                if self.is_enable and not self.is_ended:
                    self.pub_target_location.publish(self.target_location)
                    self.is_enable = False
                    self.is_ended = False
            if self.start_cmd:
                rospy.loginfo("SeekOwn end")
                self.task_counter += 1
                return 'done'
            self.r.sleep()
            
class RelWork(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['second','own','com'])
        
        self.pub_pmp_state = rospy.Publisher("pmp_state",Int8,queue_size=100)
        self.pub_stepper_state = rospy.Publisher("stepper_state",Int8,queue_size=100)
        
        self.pmp_state = Int8(data = 0)
        self.stepper_state = Int8(data = 1)
        self.task_counter = 0
        
        self.start_cmd = False
        self.back_cmd  = False
        
        self.r1 = rospy.Rate(0.5)
        self.r2 = rospy.Rate(10)

    def start_cmd_callback(self,msg):
        self.start_cmd = True
        
    def back_cmd_callback(self,msg):
        self.back_cmd = True
        
    def execute(self, ud):
        rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size=100)
        rospy.Subscriber("back_cmd",Empty,self.back_cmd_callback,queue_size=100)
        
        self.start_cmd = False
        self.back_cmd = False

        self.pub_stepper_state.publish(self.stepper_state)
        self.r1.sleep()
        self.pub_pmp_state.publish(self.pmp_state)

        while not rospy.is_shutdown():
            if self.start_cmd:
                if self.task_counter % 4 == 1:
                    self.task_counter += 1
                    return 'second'
                else:
                    self.task_counter += 1
                    return 'own'
            if self.back_cmd:
                if self.task_counter % 4 == 1:
                    self.task_counter += 1
                    return 'second'
                else:
                    self.task_counter += 1
                    return 'com'
            self.r2.sleep()

class Terminal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

    def execute(self, ud):
        while not rospy.is_shutdown():
            rospy.loginfo("nyan")
        return 'done'

if __name__ == '__main__':
    rospy.init_node("task_manager")
    field_color = rospy.get_param("~field_color")

    if field_color == "blue":
        own_arm_pos_x = [0.435,0.435,0.235,0.035,0.035,-0.165,-0.365,-0.365]
        own_arm_pos_y = [0.448,0.448,0.448,0.448,0.448,0.448,0.448,0.448]
        com_arm_pos_x = [0.595,0.455,0.315,0.175,0.035,-0.105,-0.245,-0.385,-0.525]
        com_arm_pos_y = [0.745, 0.745, 0.745, 0.745, 0.745, 0.745, 0.745, 0.745]
        box_pos_x = [ 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685]
        box_pos_y = [-0.537,-0.437,-0.337,-0.437,-0.336,-0.236,-0.136,-0.236,-0.135,-0.035, 0.065,-0.035]
    elif field_color == "red":
        own_arm_pos_x = [0.435,0.435,0.235,0.035,0.035,-0.165,-0.365,-0.365]
        own_arm_pos_y = [-0.448,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448]
        com_arm_pos_x = [0.595,0.455,0.315,0.175,0.035,-0.105,-0.245,-0.385,-0.525]
        com_arm_pos_y = [-0.745, -0.745, -0.745, -0.745, -0.745, -0.745, -0.745, -0.745]
        box_pos_x = [0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685]
        box_pos_y = [0.537, 0.437, 0.337, 0.437, 0.336, 0.236, 0.136, 0.236, 0.135, 0.035,-0.065, 0.035]
    else:
        rospy.loginfo("invalid field_color f**k you!")

    rospy.Subscriber('jaguar_position',Float32MultiArray,jaguar_position_callback,queue_size = 100)
    sm_top = smach.StateMachine(outcomes=['end'])
    with sm_top:
        smach.StateMachine.add('Init',Init(),transitions={'done':'SeekOwn'})
        smach.StateMachine.add('SeekOwn',SeekOwn(),transitions={'done':'GrabOwn','completed':'Terminal'})
        smach.StateMachine.add('GrabOwn',GrabOwn(),transitions={'done':'SeekBox'})
        smach.StateMachine.add('SeekCom',SeekCom(),transitions={'done':'GrabCom','completed':'Terminal'})
        smach.StateMachine.add('GrabCom',GrabCom(),transitions={'done':'SeekBox','second':'SeekCom'})
        smach.StateMachine.add('SeekBox',SeekBox(),transitions={'done':'RelWork'})
        smach.StateMachine.add('RelWork',RelWork(),transitions={'second':'SeekBox','own':'SeekOwn','com':'SeekCom'})
        smach.StateMachine.add('Terminal',Terminal(),transitions={'done':'end'})
    
    sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
    sis.start()
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
    
    
# #!/usr/bin/env python3

# from std_msgs.msg import Int8MultiArray
# from std_msgs.msg import Empty
# from std_msgs.msg import Bool
# from std_msgs.msg import Int8
# from std_msgs.msg import Float32MultiArray
# import rospy
# import smach
# import smach_ros
# import math

# arm_pos_x = [0,0,0,0,0,0,0,0,0,0]
# arm_pos_y = [0,0,0,0,0,0,0,0,0,0]
# box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
# box_pos_y    = [0,0,0,0,0,0,0,0,0,0]

# def cal_dist(x0:float,y0:float,x1:float,y1:float):
#     return math.sqrt((x1-x0)**2+(y1-y0)**2)

# def jaguar_position_callback(msg):
#     global arm_pos_x
#     global arm_pos_y

#     tag = 0
#     min_dist = 100

#     if not len(msg.data) == 0 or (msg.data[0] == 0 and msg.data[1] == 0):
#         for i in range(10):
#             dist = cal_dist(msg.data[0],msg.data[1],arm_pos_x[i],arm_pos_y[i])
#             if dist < min_dist:
#                 min_dist = dist
#                 tag = i

#         arm_pos_x[tag] = msg.data[0]
#         arm_pos_y[tag] = msg.data[1]

# class Task1_Init(smach.State): #諸々の初期化待機
#     def __init__(self):
#         self.field_color = rospy.get_param("~field_color")
#         rospy.loginfo(self.field_color)
#         self.is_Handy = True

#         self.is_started = False
#         self.auto_flag  = False
#         smach.State.__init__(self,outcomes=['done'])

#         self.pub_target_location     = rospy.Publisher("target_location",Float32MultiArray,queue_size = 100)
#         self.pub_servo_cmd           = rospy.Publisher("servo_cmd", Int8, queue_size=100)
#         self.target_location = Float32MultiArray()
#         self.servo_cmd = Int8(data=0)

#         if self.field_color == "blue":
#             self.target_location.data = [0.3 * math.sqrt(3),0]
#         elif self.field_color == "red":
#             rospy.loginfo("nya")
#             self.target_location.data = [0.3 * math.sqrt(3),0]

#         rospy.Subscriber("start_cmd",Empty,self.start_cmd_callback,queue_size = 100)
#         rospy.Subscriber("is_handy",Bool,self.is_handy_callback,queue_size=100)
#         self.r = rospy.Rate(10)

#     def start_cmd_callback(self,msg):
#         self.is_started = not self.is_started

#     def is_handy_callback(self,msg):
#         self.is_Handy = msg.data
#         if self.is_Handy:
#             self.auto_flag = False

#     def execute(self, ud):
#         while not rospy.is_shutdown():
#             if not self.is_Handy and not self.auto_flag:
#                 self.pub_target_location.publish(self.target_location)
#                 self.auto_flag = True
#             if not self.is_Handy:
#                 self.pub_servo_cmd.publish(self.servo_cmd)
#             if self.is_started:
#                 rospy.loginfo("task_manager : Task1_Init is ended")
#                 return 'done'
#             self.r.sleep()

# class Task2_SeekWork(smach.State): #じゃがりこ探しに行く
#     def __init__(self):
#         smach.State.__init__(self,outcomes=['done','completed'])
#         global arm_pos_x
#         global arm_pos_y
#         global box_pos_x
#         global box_pos_y

#         self.stepper_state  = Int8(data = 0)
#         self.servo_cmd      = Int8(data = 0)

#         self.end_flag       = False
#         self.task_counter   = 0

#         self.r = rospy.Rate(10)

#     def end_cmd_callback(self,msg):
#         rospy.loginfo("task_manager ended nyanyanyanyanyanya")
#         self.end_flag = True

#     def grab_cmd_callback(self,msg):
#         rospy.loginfo("grab")
#         self.end_flag = True

#     def execute(self, ud):
#         rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size = 100)
#         rospy.Subscriber("grab_cmd",Empty,self.grab_cmd_callback,queue_size = 100)
#         self.pub_servo_cmd  = rospy.Publisher("servo_cmd",Int8,queue_size=100)
#         self.target_pub     = rospy.Publisher("target_location",Float32MultiArray,queue_size = 100)
#         self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=100)

#         rospy.loginfo("task_manager : Task2_SeekWork is activated")
#         if self.task_counter < 10 :
#             if self.is_Handy == False:
#                 self.target_msg = Float32MultiArray()
#                 self.target_msg.data = [arm_pos_x[self.task_counter],arm_pos_y[self.task_counter]]
#                 self.target_pub.publish(self.target_msg)

#             if self.task_counter == 1 or self.task_counter == 2:
#                 self.stepper_state.data = 3
#             else:
#                 self.stepper_state.data = 1
#             self.stepper_state_pub.publish(self.stepper_state)
#             self.end_flag=False

#             while not rospy.is_shutdown():
#                 if not self.is_Handy:
#                     if self.task_counter == 1 or self.task_counter == 2:
#                         self.servo_cmd.data = 1
#                     else :
#                         self.servo_cmd.data = 0
#                     self.pub_servo_cmd.publish(self.servo_cmd)

#                 if self.end_flag:
#                     rospy.loginfo("task_manager : Task2_SeekWork is ended")
#                     self.task_counter = self.task_counter + 1
#                     self.end_flag = False
#                     return 'done'

#                 rospy.loginfo("nya")

#                 self.r.sleep()
#         else :
#             rospy.loginfo("3nya")
#             rospy.loginfo("task_manager : all tasks completed")
#             self.task_counter = 0
#             return 'completed'

# class Task3_GrabWork(smach.State):
#     def __init__(self):
#         rospy.loginfo("task_manager : Task3_GrabWork is activated")
#         smach.State.__init__(self, outcomes=['done'])
#         rospy.Subscriber("is_grabbed",Int8MultiArray,self.is_grabbed_callback,queue_size=1)
#         rospy.Subscriber("step_cmd",Bool,self.step_cmd_callback,queue_size=1)
#         self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=1)
#         self.stepper_state = Int8(data = 0)
#         self.task_counter   = 0
#         self.arm_hight      = 0
#         self.jaguar_hight   = 90
#         self.is_completed   = False
#         self.handstate      = [0,0]
#         self.r              = rospy.Rate(30)
#         self.loop_counter   = 0

#     def step_cmd_callback(self,msg):
#         if is_Handy == True:
#             if msg.data == True:#上昇
#                 if self.stepper_state == 3:
#                     self.stepper_state = 0
#                 else:
#                     self.stepper_state = self.stepper_state - 1
#         #     if self.stepper_state == 2:
#         #         self.stepper_state = 1
#         #     elif self.stepper_state == 4:
#         #         self.stepper_state.data = 3
#         #     elif self.stepper_state == 1 or self.stepper_state == 3:
#         #         self.stepper_state.data = 0

#             elif msg.data == False:#下降
#                 if self.stepper_state == 0 and (self.task_counter == 1 or self.task_counter ==2):
#                 # if self.task_counter == 1 or self.task_counter == 2:
#                     self.stepper_state = 3
#                 # else:
#                 #     self.stepper_state = 1
#                 else:
#                     self.stepper_state = self.stepper_state + 1
#             # elif self.stepper_state == 1:
#             #     self.stepper_state = 2
#             # elif self.stepper_state == 3:
#             #     self.stepper_state = 4
#             self.stepper_state_pub.publish(self.stepper_state)

#     def is_grabbed_callback(self,msg):
#         self.handstate = msg.data
#         if self.task_counter < 7:
#             if msg.data[0] == 1 and msg.data[1] == 1 and self.arm_hight > self.jaguar_hight:
#                 self.is_completed = True
#             else :
#                 self.is_completed = False
#         else:
#             if (msg.data[0] == 1 or msg.data[1] == 1) and self.arm_hight > self.jaguar_hight:
#                 self.is_completed = True
#             else :
#                 self.is_completed = False

#     def execute(self, ud):
#         while not rospy.is_shutdown():
#             if self.loop_counter == 0:
#                 if self.task_counter == 1 or self.task_counter == 2:
#                     self.stepper_state.data = 4
#                 else:
#                     self.stepper_state.data = 2
#             elif self.loop_counter == 60:
#                  self.stepper_state.data = 0
#                  self.loop_counter = 0

#             self.stepper_state_pub.publish(self.stepper_state)

#             if self.is_completed and self.loop_counter > 70:
#                 self.task_counter = self.task_counter + 1
#                 self.loop_counter = 0
#                 if self.task_counter > 9:
#                     self.task_counter = 0
#                 return 'done'
#             elif not self.is_completed and self.loop_counter > 70:
#                 self.loop_counter = 0
#             self.loop_counter = self.loop_counter + 1
#             self.r.sleep()

# class Task4_SeekBox(smach.State):
#     def __init__(self):
#         rospy.loginfo("task_manager : Task4_SeekBox is activated")
#         smach.State.__init__(self,outcomes=['done'])
#         rospy.Subscriber("release_cmd",Empty,queue_size = 1)
#         rospy.Subscriber("end_cmd",Empty,self.end_cmd_callback,queue_size=1)
#         rospy.Subscriber("step_cmd",Bool,self.step_cmd_callback,queue_size=1)
#         self.pub_servo_cmd  = rospy.Publisher("servo_cmd",Bool,queue_size=100)
#         self.stepper_state_pub = rospy.Publisher("stepper_state",Int8,queue_size=1)
#         self.stepper_state = Int8(data = 0)
#         self.target_pub = rospy.Publisher("target_location",Float32MultiArray,queue_size = 1)

#         self.is_released = False
#         self.task_counter = 0
#         self.r = rospy.Rate(30)
#         self.servo_cmd = Bool(data = False)

#     def release_cmd_callback(self,msg):
#         self.is_released = True

#     def end_cmd_callback(self,msg):
#         self.stepper_state.data == 2
#         self.stepper_state_pub.publish(self.stepper_state)

#     def step_cmd_callback(self,msg):
#         if is_Handy == True:
#             if msg.data == True:#上昇
#                 if self.stepper_state == 3:
#                     self.stepper_state = 0
#                 else:
#                     self.stepper_state = self.stepper_state - 1
#         #     if self.stepper_state == 2:
#         #         self.stepper_state = 1
#         #     elif self.stepper_state == 4:
#         #         self.stepper_state.data = 3
#         #     elif self.stepper_state == 1 or self.stepper_state == 3:
#         #         self.stepper_state.data = 0

#             elif msg.data == False:#下降
#                 if self.stepper_state == 0 and (self.task_counter == 1 or self.task_counter ==2):
#                 # if self.task_counter == 1 or self.task_counter == 2:
#                     self.stepper_state = 3
#                 # else:
#                 #     self.stepper_state = 1
#                 else:
#                     self.stepper_state = self.stepper_state + 1
#             # elif self.stepper_state == 1:
#             #     self.stepper_state = 2
#             # elif self.stepper_state == 3:
#             #     self.stepper_state = 4
#             self.stepper_state_pub.publish(self.stepper_state)

#     def execute(self, ud):
#         global arm_pos_x
#         global arm_pos_y
#         global box_pos_x
#         global box_pos_y
#         global is_Handy

#         if is_Handy == False:
#             self.target_msg = Float32MultiArray()
#             self.target_msg.data = [box_pos_x[self.task_counter],
#                                         box_pos_y[self.task_counter]]

#         self.target_pub.publish(self.target_msg)

#         while not rospy.is_shutdown():
#             if not is_Handy:
#                 if self.task_counter == 6 or self.task_counter == 7:
#                     self.servo_cmd = True
#                 else:
#                     self.servo_cmd = False
#                 self.pub_servo_cmd.publish(self.servo_cmd)
#             if self.is_released:
#                 rospy.loginfo("task_manager : Task4_SeekBox is ended")
#                 self.is_released = False
#                 self.task_counter = self.task_counter + 1
#                 return 'done'
#             self.r.sleep()

# def main():
#     rospy.Subscriber('jaguar_position',Float32MultiArray,jaguar_position_callback,queue_size = 100)

#     sm_top = smach.StateMachine(outcomes=['succeeded'])
#     with sm_top:
#         smach.StateMachine.add('Init',Task1_Init(),transitions={'done':'SeekWork'})
#         smach.StateMachine.add('SeekWork',Task2_SeekWork(),transitions={'done':'GrabWork','completed':'succeeded'})
#         smach.StateMachine.add('GrabWork',Task3_GrabWork(),transitions={'done':'SeekBox'})
#         smach.StateMachine.add('SeekBox',Task4_SeekBox(),transitions={'done':'SeekWork'})

#     sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
#     sis.start()
#     outcome = sm_top.execute()
#     rospy.spin()
#     sis.stop()

# if __name__ == '__main__':
#     rospy.init_node("task_manager")
#     field_color = rospy.get_param("~field_color")

#     if field_color == "blue":
#         arm_pos_x = [0.435,0.595,0.455,0.435,0.235,0.035,0.035,-0.165,-0.365,-0.365]
#         arm_pos_y = [0.448,0.745,0.745,0.448,0.448,0.448,0.448,0.448,0.448,0.448]
#         box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
#         box_pos_y    = [0,0,0,0,0,0,0,0,0,0]
#     elif field_color == "red":
#         arm_pos_x = [0.435,0.595,0.455,0.435,0.235,0.035,0.035,-0.165,-0.365,-0.365]
#         arm_pos_y = [-0.448,-0.745,-0.745,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448]
#         box_pos_x    = [0,0,0,0,0,0,0,0,0,0]
#         box_pos_y    = [0,0,0,0,0,0,0,0,0,0]
#     else:
#         rospy.loginfo("invalid field_color f**k you!")

#     main()
