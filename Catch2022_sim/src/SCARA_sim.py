from rosdep2 import RosdepInternalError
import rospy
from sensor_msgs.msg import JointState
import math

def simulator():
    rospy.init_node("SCARA_sim")
    rviz_pub = rospy.Publisher("joint_states",JointState,queue_size=100)

    r = rospy.Rate(10)
    rviz_msg = JointState()

    cnt = 0
    rviz_msg.name[0] = "stand_arm1"
    rviz_msg.name[1] = "arm1_arm2"
    rviz_msg.name[2] = "arm2_linear"
    rviz_msg.name[3] = "linear_wrist"

    while not rospy.is_shutdown():
        rviz_msg.header.stamp = rospy.Time.now()

        rviz_msg.position[0] = float(cnt)/50
        rviz_msg.position[1] = float(cnt)/50
        rviz_msg.position[2] = float(cnt)/-1000
        rviz_msg.position[3] = float(cnt)/50

        rviz_pub.publish(rviz_msg)
        cnt = cnt + 1
        r.sleep()

if __name__ == "__main__" :
    try:
        simulator()
    
    except RosdepInternalError: pass