#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from math import pi
import math

class Scara():
    def __init__(self):
        self.sign = 1#青まいなす,赤ぷらす
        self.work_flag = False
        self.l1     = 0.6
        self.l2     = 0.3
        self.a      = 1
        self.h      = 10
        self.r      = rospy.Rate(10)
        self.target = np.array([[0],[0]])
        self.current = np.array([[0],[0.52]])
        self.current_theta = np.array([[pi/3],[pi/1.5]])
        self.start = np.array([[0],[0]])
        self.J=np.array([[1,0],[0,1]])
        self.pub_move_cmd = rospy.Publisher("move_cmd",Float32MultiArray,queue_size = 100)
        rospy.Subscriber("current_angle",Float32MultiArray,self.current_angle_callback,queue_size = 100)
        rospy.Subscriber("target_location",Float32MultiArray,self.target_location_callback,queue_size = 100),
        rospy.Subscriber("is_blue",Bool,self.color_callback,queue_size=100)
        self.update()
        
    def color_callback(self,msg):
        if msg.data == False:
            self.sign = 1
        elif msg.data == True:
            self.sign = -1
                

    def target_location_callback(self,msg):
        rospy.loginfo("nya")
        self.target = np.array([[msg.data[0]],[msg.data[1]]])
        self.work_flag = True
        self.er = self.target - self.start
        self.distance=np.linalg.norm(self.er)
        self.t0 = math.sqrt(self.distance/self.a)
        self.end_theta1 = math.atan(self.target[1,0]/self.target[0,0])+self.sign*math.acos((np.linalg.norm(self.target)**2+self.l1**2-self.l2**2)/(2*self.l1*np.linalg.norm(self.target)))
        self.end_theta2 = -self.sign*(pi-math.acos((self.l1**2+self.l2**2-np.linalg.norm(self.target)**2)/(2*self.l1*self.l2)))
        self.end_theta = np.array([[self.end_theta1],[self.end_theta2]])
    
    def current_angle_callback(self,msg):
        self.current_theta1 = (msg.data[0]-35) * pi / 180
        self.current_theta2 = (msg.data[1]-138) * pi / 180
        self.current_theta = np.array([[self.current_theta1],[self.current_theta2]])
        self.current_x = self.l1 * math.cos (self.current_theta1) + self.l2 * math.cos (self.current_theta1 + self.current_theta2)
        self.current_y = self.l1 * math.sin (self.current_theta1) + self.l2 * math.sin (self.current_theta1 + self.current_theta2)
        self.current = np.array([[self.current_x],[self.current_y]])
        self.J = np.matrix([[self.poi(-self.current_y),self.poi(-self.l2*math.sin(self.current_theta1+self.current_theta2))],[self.poi(self.current_x),self.poi(self.l2*math.cos(self.current_theta1+self.current_theta2))]])
        # rospy.loginfo("J")
        # rospy.loginfo(self.J)
        if not self.work_flag:
            self.start = self.current
    
    def poi(self,arg):
        if math.fabs(arg) < 0.001:
            return 0
        else :
            return arg

    def update(self):
        cnt = 1
        while not rospy.is_shutdown():
            self.next_theta = np.array([[0],[0]])
            if self.work_flag == True:
                if cnt <= self.h * self.t0:
                    next_dist = self.a*(2*cnt-1)/(2*(self.h**2))
                    self.next_r = (next_dist/self.distance)*self.er
                    self.next_theta = np.dot(np.linalg.inv(self.J),next_r) 
                    cnt = cnt + 1
                elif cnt <= self.h * self.t0*2:
                    next_dist = (self.a/(2*self.h))*(4*self.t0-((2*cnt-1)/self.h))
                    next_r = (next_dist/self.distance)*self.er    
                    self.next_theta = np.dot(np.linalg.inv(self.J),next_r)
                    cnt = cnt + 1
                else:
                    self.work_flag = False
                    cnt = 1
                    self.pub_move_cmd.publish(data=[self.end_theta1,self.end_theta2])
                    rospy.loginfo("end")
            
            # self.target_theta = (180/math.pi)*(self.next_theta+self.current_theta)+np.array([[35],[138]])
            self.next_pos =  (self.next_r + self.current)
            # rospy.loginfo("target_theta")
            # rospy.loginfo(self.current_theta)
            f_msg = Float32MultiArray(data = [self.next_pos[0,0],self.next_pos[1,0]])
            # f_msg = Float32MultiArray(data = [self.target_theta[0,0],self.target_theta[1,0]])
            self.pub_move_cmd.publish(f_msg)    
            print(f_msg)   
            self.r.sleep()

if __name__ == "__main__":
    rospy.init_node("scara_ik")
    rospy.loginfo("scara_ik : node is activated")
    scara =Scara()
    rospy.loginfo("scara_ik : process_end")

# import sys
# import copy
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg
# from math import pi
# from std_msgs.msg import String
# from moveit_commander.conversions import pose_to_list
# from geometry_msgs.msg import Quaternion, Vector3

# def all_close(goal, actual, tolerance):
#   """
#   Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
#   @param: goal       A list of floats, a Pose or a PoseStamped
#   @param: actual     A list of floats, a Pose or a PoseStamped
#   @param: tolerance  A float
#   @returns: bool
#   """
#   all_equal = True
#   if type(goal) is list:
#     for index in range(len(goal)):
#       if abs(actual[index] - goal[index]) > tolerance:
#         return False

#   elif type(goal) is geometry_msgs.msg.PoseStamped:
#     return all_close(goal.pose, actual.pose, tolerance)

#   elif type(goal) is geometry_msgs.msg.Pose:
#     return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
#   print(actual)
#   return True



# class MoveGroupPythonIntefaceTutorial(object):
#   """MoveGroupPythonIntefaceTutorial"""
#   def __init__(self):
#     super(MoveGroupPythonIntefaceTutorial, self).__init__()

#     ## BEGIN_SUB_TUTORIAL setup
#     ##
#     ## First initialize `moveit_commander`_ and a `rospy`_ node:
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#     ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
#     ## kinematic model and the robot's current joint states
#     self.robot = moveit_commander.RobotCommander()

#     ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
#     ## for getting, setting, and updating the robot's internal understanding of the
#     ## surrounding world:
#     scene = moveit_commander.PlanningSceneInterface()

#     ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
#     ## to a planning group (group of joints).  In this tutorial the group is the primary
#     ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
#     ## If you are using a different robot, change this value to the name of your robot
#     ## arm planning group.
#     ## This interface can be used to plan and execute motions:
#     group_name = "arm"
#     move_group = moveit_commander.MoveGroupCommander(group_name)

#     ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
#     ## trajectories in Rviz:
#     display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#                                                    moveit_msgs.msg.DisplayTrajectory,
#                                                    queue_size=20)

#     ## END_SUB_TUTORIAL

#     ## BEGIN_SUB_TUTORIAL basic_info
#     ##
#     ## Getting Basic Information
#     ## ^^^^^^^^^^^^^^^^^^^^^^^^^
#     # We can get the name of the reference frame for this robot:
#     planning_frame = move_group.get_planning_frame()
#     # print "============ Planning frame: %s" % planning_frame

#     # We can also print the name of the end-effector link for this group:
#     eef_link = move_group.get_end_effector_link()
#     # print "============ End effector link: %s" % eef_link

#     # We can get a list of all the groups in the robot:
#     group_names = self.robot.get_group_names()
#     # print "============ Available Planning Groups:", robot.get_group_names()

#     # Sometimes for debugging it is useful to print the entire state of the
#     # robot:
#     # print "============ Printing robot state"
   
#     # print ""
#     ## END_SUB_TUTORIAL

#     # Misc variables
#     self.box_name = ''
#     # self.robot = robot
#     self.scene = scene
#     self.move_group = move_group
#     self.display_trajectory_publisher = display_trajectory_publisher
#     self.planning_frame = planning_frame
#     self.eef_link = eef_link
#     self.group_names = group_names


#   def go_to_pose_goal(self):
#     # Copy class variables to local variables to make the web tutorials more clear.
#     # In practice, you should use the class variables directly unless you have a good
#     # reason not to.
#     move_group = self.move_group

#     ## BEGIN_SUB_TUTORIAL plan_to_pose
#     ##
#     ## Planning to a Pose Goal
#     ## ^^^^^^^^^^^^^^^^^^^^^^^
#     ## We can plan a motion for this group to a desired pose for the
#     ## end-effector:
#     pose_goal = geometry_msgs.msg.Pose()
#     pose_goal.position = Vector3(0.7, 0.0, 0.7)
#     # pose_goal.orientation.w = 1.0
#     # pose_goal.position.x = 0.5
#     # pose_goal.position.y = 0.6
#     # pose_goal.position.z = 0.3

#     move_group.set_pose_target(pose_goal)

#     ## Now, we call the planner to compute the plan and execute it.
#     plan = move_group.go(wait=True)
#     # Calling `stop()` ensures that there is no residual movement
#     move_group.stop()
#     # It is always good to clear your targets after planning with poses.
#     # Note: there is no equivalent function for clear_joint_value_targets()
#     move_group.clear_pose_targets()

#     ## END_SUB_TUTORIAL

#     # For testing:
#     # Note that since this section of code will not be included in the tutorials
#     # we use the class variable rather than the copied state variable
#     current_pose = self.move_group.get_current_pose().pose
#     return all_close(pose_goal, current_pose, 0.01)

# def main():
#     tutorial = MoveGroupPythonIntefaceTutorial()
#     tutorial.go_to_pose_goal()

# if __name__ == '__main__':
#   main()


# import sys
# import rospy
# import moveit_commander
# import moveit_msgs.msg
# import geometry_msgs.msg

# from math import pi

# def talker():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('inverse_node',
#                     anonymous=True)
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group_name = "arm"
#     move_group = moveit_commander.MoveGroupCommander(group_name)
#     move_group.get_planner_id()
#     # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
#     #                                            moveit_msgs.msg.DisplayTrajectory,
#     #                                            queue_size=20)
#     # set position & orientation
#     pose_goal = geometry_msgs.msg.Pose()
#     pose_goal.position.x = 0.8
#     pose_goal.position.y = 0.8
#     pose_goal.position.z = 0
#     pose_goal.orientation.x = 0
#     pose_goal.orientation.y = 0
#     pose_goal.orientation.z = 0
#     pose_goal.orientation.w = 0

#     move_group.set_pose_target(pose_goal)
#     plan = move_group.go(wait=True) 
#     rospy.loginfo(plan)
#     move_group.stop()
#     move_group.clear_pose_targets()
    
#     print(move_group.get_current_joint_values())
    
#     r = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():

#         r.sleep()

# if __name__ == '__main__':
#     try:
#         talker() 
#     except rospy.ROSInterruptException: pass





# import sys
# import rospy
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import JointState
# from moveit_msgs.msg import Constraints
# from moveit_msgs.msg import RobotState
# from moveit_msgs.msg import PositionIKRequest
# from moveit_msgs.srv import GetPositionIK

# if __name__ == '__main__':
#     rospy.init_node('get_position_ik')

#     # joint_state = rospy.wait_for_message('joint_states',JointState) #現在の各関節状態をサブ
#     joint_state = JointState()
#     joint_state.header.frame_id = 'base_link'
#     joint_state.name = ['stand_arm1','arm1_arm2','arm2_linear','linear_wrist']
#     joint_state.position = [0,0,0,0]

#     print(joint_state)
    
#     robot_state = RobotState()
#     robot_state.joint_state = joint_state #jointstate型からrobotstate型変換

#     pose_stamped = PoseStamped()
#     pose_stamped.header.frame_id = 'base_link'
#     pose_stamped.pose.position.x = 0.6
#     pose_stamped.pose.position.y = 0.7
#     pose_stamped.pose.position.z = 0.0
#     pose_stamped.pose.orientation.x = 0.0
#     pose_stamped.pose.orientation.y = 0.0
#     pose_stamped.pose.orientation.z = 0.0
#     pose_stamped.pose.orientation.w = 0.0

#     constraints = Constraints() #制約今回は空

#     try:
#         request = PositionIKRequest()
#         request.group_name = 'arm' #setup assistantで設定したplanning group
#         request.robot_state = robot_state #現在の各関節の状態
#         request.constraints = constraints #
#         request.pose_stamped = pose_stamped #目標

#         compute_ik = rospy.ServiceProxy('/compute_ik',GetPositionIK)
#         response = compute_ik(request)
#         print(response)
    
#     except rospy.ServiceException as e:
#         print("Service call failed: {}".format(e))

# import sys
# import moveit_commander
# import rospy


# def main():
#     # moveit_commanderの初期化
#     moveit_commander.roscpp_initialize(sys.argv)

#     # ノードの初期化
#     rospy.init_node('robot_info')

#     # RobotCommanderのインスタンス化
#     robot = moveit_commander.RobotCommander()

#     # MoveGroupCommanderのインスタンス化
#     move_group = moveit_commander.MoveGroupCommander("arm")

#     # ロボットのプランニングフレーム名の取得
#     planning_frame = move_group.get_planning_frame()
#     print ("planning_frame: %s" % planning_frame)

#     # エンドエフェクタリンク名の取得
#     end_effector_link = move_group.get_end_effector_link()
#     print ("end_effector_link: %s" % end_effector_link)

#     # ロボット内のグループ名のリストの取得
#     group_names = robot.get_group_names()
#     print ("group_names:", group_names)

#     # ロボットの現在の状態の取得
#     current_state = robot.get_current_state()
#     print ("\n", current_state)

#     # ノード終了まで待機
#     rospy.spin()


# if __name__ == "__main__":
#     main()