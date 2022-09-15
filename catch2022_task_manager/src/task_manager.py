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

is_handy = True
past_is_handy = False
is_enable = True
is_ended = True
start_cmd = False
is_connected = False


own_jaguar_pos_x = [0.535, 0.435, 0.435, 0.335, 0.235, 0.235, 0.135, 0.035, 0.035, -0.065, -0.165, -0.165, -0.265, -0.365, -0.365, -0.465]
own_jaguar_pos_y = [-0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448, -0.348, -0.548, -0.448]
com_jaguar_pos_x = [0.595, 0.455, 0.315, 0.175, 0.035, -0.105, -0.245, -0.385, -0.525]
com_jaguar_pos_y = [-0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845, -0.845]
box_jaguar_pos_x = [0.585, 0.685, 0.785, 0.585, 0.685, 0.785, 0.585, 0.685, 0.785, 0.585, 0.685, 0.785,0.585, 0.685, 0.785, 0.585, 0.685, 0.785]
box_jaguar_pos_y = [0.029, 0.029, 0.029, 0.129, 0.129, 0.129, 0.230, 0.230, 0.230, 0.330, 0.330, 0.330, 0.431, 0.431, 0.431, 0.531, 0.531, 0.531]

pub_target_location = rospy.Publisher("move_cmd",Float32MultiArray,queue_size=100)
pub_servo_cmd       = rospy.Publisher("servo_cmd",Int8,queue_size=100)
pub_pmp_state       = rospy.Publisher("pmp_state",Int8,queue_size=100)
pub_stepper_state   = rospy.Publisher("stepper_state",Int8,queue_size=100)
pub_led_hsv         = rospy.Publisher("led_hsv",Int16MultiArray,queue_size=100)

def p_target_location(x:float,y:float):
    global pub_target_location
    
    f_msg=Float32MultiArray(data=[x,y])
    pub_target_location.publish(f_msg)
    
def p_servo_cmd (x:Int8):
    global pub_servo_cmd
    
    i_msg=Int8(data=x)
    pub_servo_cmd.publish(i_msg)

def p_pmp_state(x: Int8):
    global pub_pmp_state

    i_msg = Int8(data=x)
    pub_pmp_state.publish(i_msg)
    rospy.loginfo("pub")

def p_stepper_state(x: Int8):
    global pub_stepper_state

    i_msg = Int8(data=x)
    pub_stepper_state.publish(i_msg)

def p_led_hsv(x:int):
    global pub_led_hsv

    i_msg = Int16MultiArray(data=[x,255,255])
    pub_led_hsv.publish(i_msg)


def is_handy_callback(msg):  # 手動or自動
    global is_handy
    global is_enable
    global is_ended
    past_is_handy = is_handy
    is_handy = msg.data
    if past_is_handy and not is_handy:
        is_enable = True
        is_ended = False
        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/AUTO.wav")
    elif not past_is_handy and is_handy:
        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/Hand.wav")


def start_cmd_callback(msg):  # Stateの遷移コマンド
    global start_cmd
    start_cmd = True


def end_cmd_callback(msg):  # 移動の終了コマンド
    global is_ended
    is_ended = True


def is_connected_callback(msg):  # マイコンとの接続確認
    global is_connected
    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/CONNECT.wav")
    is_connected = True

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

    own_tag = 0
    com_tag = 0
    min_own_dist = 100
    min_com_dist = 100

    if not len(msg.data) == 0 or (msg.data[0] == 0 and msg.data[1] == 0):
        for i in range(16):
            dist = cal_dist(msg.data[0],msg.data[1],own_jaguar_pos_x[i],own_jaguar_pos_y[i])
            if dist < 350 and dist < min_own_dist:
                min_own_dist = dist
                own_tag = i

        for i in range(9):
            dist = cal_dist(msg.data[0],msg.data[1],com_jaguar_pos_x[i],com_jaguar_pos_y[i])
            if dist < 350 and dist < min_com_dist:
                min_com_dist = dist
                com_tag = i

        if min_own_dist > min_com_dist: 
            com_jaguar_pos_x[com_tag] = msg.data[0]
            com_jaguar_pos_y[com_tag] = msg.data[1]
            com_arm_pos_x[com_tag] = msg.data[0]
            com_arm_pos_y[com_tag] = msg.data[1] - (msg.data[1]*0.1)/math.fabs(msg.data[1])
        else:
            own_jaguar_pos_x[com_tag] = msg.data[0]
            own_jaguar_pos_y[com_tag] = msg.data[1]
            num = [1,0,0,1,2,2,4,3,3,4,5,5,7,6,6,7]
            if own_tag %3 == 1:
                own_arm_pos_x[num[own_tag]] = (own_jaguar_pos_x[own_tag]+own_jaguar_pos_x[own_tag+1])/2
                own_arm_pos_y[num[own_tag]] = (own_jaguar_pos_y[own_tag]+own_jaguar_pos_y[own_tag+1])/2
            elif own_tag %3 == 2:
                own_arm_pos_x[num[own_tag]] = (own_jaguar_pos_x[own_tag]+own_jaguar_pos_x[own_tag-1])/2
                own_arm_pos_y[num[own_tag]] = (own_jaguar_pos_y[own_tag]+own_jaguar_pos_y[own_tag-1])/2
            elif own_tag %6 == 0:
                own_arm_pos_x[num[own_tag]] = (own_jaguar_pos_x[own_tag]+own_jaguar_pos_x[own_tag+3])/2
                own_arm_pos_y[num[own_tag]] = (own_jaguar_pos_y[own_tag]+own_jaguar_pos_y[own_tag+3])/2
            elif own_tag %6 == 3:
                own_arm_pos_x[num[own_tag]] = (own_jaguar_pos_x[own_tag]+own_jaguar_pos_x[own_tag-3])/2
                own_arm_pos_y[num[own_tag]] = (own_jaguar_pos_y[own_tag]+own_jaguar_pos_y[own_tag-3])/2


            


# ここからステートマシン
class Connect(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.pub_connect_device = rospy.Publisher('connect_device',Empty,queue_size=100)
        self.r = rospy.Rate(10)

    def execute(self, ud):
        global is_connected
        global start_cmd
        self.pub_device_start = rospy.Publisher('device_start', Empty, queue_size=100)
        rospy.sleep(2)
        self.pub_connect_device.publish()
        rospy.loginfo("connect_device published")
        while not rospy.is_shutdown():
            rospy.loginfo("%d,%d",is_connected,start_cmd)
            if is_connected and start_cmd:
                self.pub_device_start.publish()
                start_cmd = False
                return 'done'
            
            if start_cmd:
                start_cmd = False
                self.pub_device_start.publish()
                return 'done'
            self.r.sleep()


class Init(smach.State): #諸々の初期化待機
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        

        self.field_color = rospy.get_param("~field_color")

        self.target_location = [0,0]
        
        if self.field_color == "blue":
            self.target_location = [0.3 * math.sqrt(3),0]
        elif self.field_color == "red":
            self.target_location = [0.3 * math.sqrt(3),0]

        self.r = rospy.Rate(10)

    def execute(self, ud):
        global is_handy
        global is_enable
        global is_ended
        global start_cmd
        
        start_cmd = False

        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/START.wav")
        
        
        while not rospy.is_shutdown():
            if not is_handy:
                if is_enable and not is_ended:
                    p_target_location(self.target_location[0],self.target_location[1])
                    p_led_hsv(0)
                    is_enable = False
                    is_ended  = False
                p_stepper_state(0)
                pub_servo_cmd.publish(0)
            if start_cmd:
                rospy.loginfo("task_manager : Task1_Init is ended")
                start_cmd = False
                is_ended = False
                return 'done'
            self.r.sleep()
            
class SeekOwn(smach.State):#自陣エリアのワークへ移動
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'completed'])
               
        self.task_counter = 0        
        self.r = rospy.Rate(10)
                
    def execute(self, ud):
        global own_arm_pos_x
        global own_arm_pos_y

        global is_handy
        global is_enable
        global is_ended
        global start_cmd
        
        if self.task_counter >= 8:
            rospy.loginfo('all task completed')
            return 'completed'
        
        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/SeekWork.wav")
        start_cmd = False
        is_enable = True
        
        while not rospy.is_shutdown():
            p_stepper_state(1)
            rospy.loginfo("%d,%d,%d",is_handy,is_enable,is_ended)
            if not is_handy:
                if self.task_counter % 3 == 1:
                    p_servo_cmd(1)
                else:
                    p_servo_cmd(0)
                if is_enable and not is_ended:
                    rospy.loginfo("%f,%f",own_arm_pos_x[self.task_counter],own_arm_pos_y[self.task_counter])
                    p_target_location(own_arm_pos_x[self.task_counter],own_arm_pos_y[self.task_counter])
                    is_enable = False
                    is_ended  = False
            if start_cmd:
                rospy.loginfo("SeekOwn end")
                start_cmd = False
                is_ended = False
                self.task_counter += 1
                return 'done'
            self.r.sleep()
    
class GrabOwn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        
        self.is_completed = False
        self.r = rospy.Rate(0.3333333)
        
    def is_grabbed_callback(self,msg):
        if msg.data == 3:
            self.is_completed = True
        else:
            self.is_completed = False
                
    def execute(self, ud):
        global start_cmd
        
        rospy.Subscriber("is_grabbed",Int8,self.is_grabbed_callback,queue_size=100)
        start_cmd = False
        self.is_completed = False
        p_pmp_state(3)
        self.counter = 1

        while not rospy.is_shutdown():
            p_stepper_state(2*(self.counter%2))
            self.r.sleep()
            if start_cmd:
                start_cmd = False
                return 'done'
            if self.is_completed == True and self.counter % 2 == 0:
                playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/get_work.wav")
                start_cmd = False
                return 'done'
            self.counter += 1

class SeekCom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done', 'completed'])

        self.task_counter = 1
        self.r = rospy.Rate(10)


    def execute(self, ud):
        global com_arm_pos_x
        global com_arm_pos_y

        global is_handy
        global is_enable
        global is_ended
        global start_cmd

        if self.task_counter >= 9:
            rospy.loginfo('all task completed')
            return 'completed'

        playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/ComArea.wav")
        start_cmd = False
        is_enable = True

        while not rospy.is_shutdown():
            p_stepper_state(0)
            if not is_handy:
                if self.task_counter % 2 == 1:
                    p_servo_cmd(2)
                else:
                    p_servo_cmd(0)
                if is_enable and not is_ended:
                    p_target_location(com_arm_pos_x[self.task_counter],com_arm_pos_y[self.task_counter])
                    is_enable = False
                    is_ended = False
            if start_cmd:
                rospy.loginfo("SeekOwn end")
                self.task_counter += 1
                start_cmd = False
                is_ended = False
                return 'done'
            self.r.sleep()
            
class GrabCom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','second'])

        self.task_counter = 0
        self.is_completed = False
        self.r = rospy.Rate(0.33)

    def is_grabbed_callback(self,msg):
        if msg.data == 3 and self.task_counter % 2 == 1:
            self.is_completed = True
        elif (msg.data == 1 or msg.data == 2) and self.task_counter % 2 == 0:
            self.is_completed = True
        else:
            self.is_completed = False
    
        
    def execute(self, ud):
        global start_cmd
        
        rospy.Subscriber("is_grabbed",Int8,self.is_grabbed_callback,queue_size=100)
        self.is_completed = False
        p_pmp_state(3)
        
        while not rospy.is_shutdown():
            p_stepper_state(4)
            self.r.sleep()
            p_stepper_state(0)
            self.r.sleep()
            if start_cmd:
                start_cmd = False
                if self.task_counter % 2 == 0 and not self.task_counter == 8:
                    self.task_counter += 1
                    return 'second'
                else:
                    self.task_counter += 1
                    return 'done'
                
            if self.is_completed == True:
                start_cmd = False
                if self.task_counter % 2 == 0 and not self.task_counter == 8:                
                    self.task_counter += 1
                    return 'second'
                else:
                    self.task_counter += 1
                    return 'done'
                    
class SeekBox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        self.task_counter = 0
        self.r = rospy.Rate(10)

    def execute(self, ud):
        global box_pos_x
        global box_pos_y
        global is_handy
        global is_enable
        global is_ended
        global start_cmd

        start_cmd = False
        is_enable = True

        while not rospy.is_shutdown():
            p_stepper_state(0)
            p_pmp_state(3)
            if not is_handy:
                if self.task_counter % 4 == 1:
                    p_servo_cmd(0)
                elif  self.task_counter % 4 == 2:
                    p_servo_cmd(2)
                else:
                    p_servo_cmd(1)                
                if is_enable and not is_ended:
                    p_target_location(box_pos_x[self.task_counter],box_pos_y[self.task_counter])
                    self.is_enable = False
                    self.is_ended = False
            if start_cmd:
                rospy.loginfo("SeekOwn end")
                self.task_counter += 1
                start_cmd = False
                is_ended = False
                return 'done'
            self.r.sleep()
            
class RelWork(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['second','own','com'])
                
        self.task_counter = 0
        
        self.back_cmd  = False
        
        self.r2 = rospy.Rate(10)

    def back_cmd_callback(self,msg):
        self.back_cmd = True
        
        
    def execute(self, ud):
        global start_cmd
        rospy.Subscriber("back_cmd",Empty,self.back_cmd_callback,queue_size=100)
        
        start_cmd = False
        self.back_cmd = False

        p_stepper_state(1)
        p_pmp_state(3)
        rospy.sleep(4)

        while not rospy.is_shutdown():
            if start_cmd:
                start_cmd = False
                if self.task_counter % 4 == 1:
                    p_pmp_state(0)
                    self.task_counter += 1
                    return 'second'
                else:
                    p_pmp_state(0)
                    self.task_counter += 1
                    return 'own'
            if self.back_cmd:
                self.back_cmd = False
                if self.task_counter % 4 == 1:
                    p_pmp_state(0)
                    self.task_counter += 1
                    return 'second'
                else:
                    p_pmp_state(0)
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
        # box_pos_y = [-0.537,-0.437,-0.337,-0.437,-0.336,-0.236,-0.136,-0.236,-0.135,-0.035, 0.065,-0.035]
        box_pos_y = [-0.029,0.071,-0.029,-0.129,-0.230,-0.130,-0.230,-0.330,-0.431,-0.331,-0.431,-0.531]
    elif field_color == "red":
        own_arm_pos_x = [0.435,0.435,0.235,0.035,0.035,-0.165,-0.365,-0.365]
        own_arm_pos_y = [-0.448,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448,-0.448]
        com_arm_pos_x = [0.595,0.455,0.315,0.175,0.035,-0.105,-0.245,-0.385,-0.525]
        com_arm_pos_y = [-0.745, -0.745, -0.745, -0.745, -0.745, -0.745, -0.745, -0.745]
        box_pos_x = [0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685, 0.685]
        # box_pos_y = [0.537, 0.437, 0.337, 0.437, 0.336, 0.236, 0.136, 0.236, 0.135, 0.035,-0.065, 0.035]
        box_pos_y = [0.029,-0.071,0.029,0.129,0.230,0.130,0.230,0.330,0.431,0.331,0.431,0.531]
    else:
        rospy.loginfo("invalid field_color f**k you!")

    rospy.Subscriber('jaguar_position',Float32MultiArray,jaguar_position_callback,queue_size = 100)
    rospy.Subscriber("is_handy", Bool, is_handy_callback, queue_size=1)
    rospy.Subscriber("start_cmd", Empty, start_cmd_callback, queue_size=100)
    rospy.Subscriber("end_cmd", Empty, end_cmd_callback, queue_size=100)
    rospy.Subscriber("is_connected", Empty, is_connected_callback, queue_size=100)


    sm_top = smach.StateMachine(outcomes=['end'])
    with sm_top:
        smach.StateMachine.add('Connect',Connect(),transitions={'done':'Init'})
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