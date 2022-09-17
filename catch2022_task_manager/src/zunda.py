#!/usr/bin/env python3
import rospy
import playsound
from std_msgs.msg import String


class Zunda():
    def __init__(self):
        rospy.Subscriber("zunda_call",String,self.zunda_callback,queue_size=100)
        
        self.zunda_shout = "zunda"
        self.shout_flag  = False
        
        self.update()
        
    def zunda_callback(self,msg):
        self.shout_flag = True
        self.zunda_shout = msg.data
    
    def update(self):
        while not rospy.is_shutdown():
            if self.shout_flag:
                if self.zunda_shout == "connect":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/CONNECT.wav")
                    self.shout_flag = False
                elif self.zunda_shout == "manual":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/Hand.wav")
                    self.shout_flag = False
                elif self.zunda_shout == "auto":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/Auto.wav")
                    self.shout_flag = False
                elif self.zunda_shout == "start":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/START.wav")
                    self.shout_flag = False
                elif self.shout_flag == "seekwork":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/SeekWork.wav")
                    self.shout_flag = False
                elif self.shout_flag == "getwork":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/get_work.wav")
                    self.shout_flag = False
                elif self.shout_flag == "comarea":
                    playsound("../catkin_ws/src/catch2022_Olympus/catch2022_task_manager/assets/ComArea.wav")
                    self.shout_flag == False
                


if __name__=='__main__':
    rospy.init_node("zunda_manager")
    arg = Zunda()