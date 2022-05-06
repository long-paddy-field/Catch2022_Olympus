#!/usr/bin/env python3
import rospy
import cv2
from cv2 import THRESH_BINARY
import tf
import geometry_msgs.msg
import math
import numpy as np
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray

jaguar_hight = 0.088
camera_angle = 55 #degree

class jaguar_indicator:

    def __init__(self,_channel,_loop_rate) -> None:
        rospy.init_node("jaguar_indicator")
        rospy.loginfo("init class")
        self.capture = cv2.VideoCapture(_channel)
        rospy.loginfo("capture image")
        # self.locate_pub = rospy.Publisher("circle_location",numpy_msg(int),queue_size=100)
#        self.tf_listener = tf.TransformListener()
        rospy.loginfo("loaded tf")
        self.capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self.wait_secs = int(1000 / self.capture.get(cv2.CAP_PROP_FPS))
        
        self.WIDTH = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.r=rospy.Rate(_loop_rate)
        self.update()        
    
    def getcircle(self):
        ret,frame = self.capture.read()
        if not ret:
            rospy.loginfo("fail to get video image")
            no_array = np.array([0,0,0])
            return no_array
        
        hsv_lower = np.array([30, 0, 0])
        hsv_upper = np.array([90, 255 ,127])
        
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv,hsv_lower,hsv_upper)
        result = cv2.bitwise_and(frame,frame,mask=hsv_mask)     
        dst_grey = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        retval,dst_bi = cv2.threshold(dst_grey,50,200,THRESH_BINARY)
#        my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=0.084/self.pic_to_m,maxRadius=0.092/self.pic_to_m)
        my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=75,maxRadius=200)

        if not isinstance(my_circles,np.ndarray) :
            rospy.loginfo("Circle is not found")
            no_array = np.array([0,0,0])
            return no_array
       
        return my_circles
    
    def update(self):
        rospy.loginfo("enter main routine")
        while not rospy.is_shutdown():
            # (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/camera', rospy.Time(0))
            # self.pic_to_m = 2*(trans[2]-jaguar_hight) * math.tan(math.radians(camera_angle/2))/self.WIDTH
            got_circle = np.array([0,0,0])
            # got_circle = self.getcircle()
            # self.locate_pub.publish(got_circle)
            rospy.loginfo("X : ",got_circle[0]," Y : ",got_circle[1]," R : ",got_circle[2])
            self.r.sleep()    
        
        self.capture.release()


if __name__ == "__main__" :
    try:
        camera_channel = rospy.get_param("~channel",4)
        loop_rate = rospy.get_param("~loop_rate",20)
        rospy.loginfo("program started!")
        func = jaguar_indicator(camera_channel,loop_rate)
    except:
        rospy.loginfo("Error: something wrong")
        pass
