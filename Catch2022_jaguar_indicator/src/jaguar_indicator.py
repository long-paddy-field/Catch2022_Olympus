#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv2 import THRESH_BINARY

class Jaguar_Indicator:
    def __init__(self,cam_ch,loop_rate):
        rospy.loginfo("jaguar_indicator : called constructor")
        rospy.init_node("jaguar_indicator")
        self.r = rospy.Rate(loop_rate)
        
        self.cv_cap= cv2.VideoCapture(cam_ch)
        self.cv_cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cv_cap.set(cv2.CAP_PROP_FPS,60)
        rospy.loginfo("jaguar_indicator : end camera setting")
        
        self.update()
        
    def getcircle(self):
        ret, frame = self.cv_cap.read()
        if ret == True:
            rospy.loginfo("success reading")
        else:
            rospy.loginfo("fail to reading")
        hsv_lower = np.array([30, 0, 0])
        hsv_upper = np.array([90, 255 ,127])
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        result = cv2.bitwise_and(frame,frame,mask=hsv_mask)
        dst_grey = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        _,dst_bi = cv2.threshold(dst_grey,50,200,THRESH_BINARY)
        # cv2.imshow("edge",dst_bi)
#        my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=0.084/self.pic_to_m,maxRadius=0.092/self.pic_to_m)
        my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=75,maxRadius=200)

        if not isinstance(my_circles,np.ndarray) :
            rospy.loginfo("Circle is not found")
            no_array = np.array([0,0,0])
            cv2.imshow("img",frame)
            return no_array
        
        # cv2.circle(frame,(200,200),10,(255,0,0),2)
        # for i in my_circles[0,:]:
        #     # draw the outer circle
        #     cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
        #     rospy.loginfo("nya")
        #     # draw the center of the circle
        #     cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

        cv2.imshow("img", frame)

        return my_circles

    def update(self):
        got_circle = list()
        while not rospy.is_shutdown():
            got_circle = self.getcircle( )
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rospy.loginfo(got_circle)
            self.r.sleep()
    
if __name__=='__main__':
    try:
        func = Jaguar_Indicator(cam_ch=4,loop_rate=50)
    except:
        rospy.logwarn("jaguar_indicator : something wrong")
    finally:
        rospy.logwarn("jaguar_indicator : end process")

# import rospy
# import cv2
# from std_msgs.msg import String
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import sys
# import numpy

# bridge = CvBridge()

# class jaguar_indicator:
#     def __init__(self):
#         jag_sub = rospy.Subscriber("camera/image_raw",Image,self.rcv_image_callback)
#         self.update()
    
#     def rcv_image_callback(self,ros_image):
#         rospy.loginfo("jaguar_indicator : recieve image")
#         global bridge
#         try:
#             self.cv_image= bridge.imgmsg_to_cv2(ros_image)
#             rospy.loginfo("jaguar_indicator : success convert")
#             # (rows, cols, channels) = self.cv_image.shape()
#         except CvBridgeError as e:
#             rospy.loginfo("jaguar_indicator : fail to convert")
        
        
#     def update(self):
#         rospy.loginfo("jaguar_indicator : enter main routine")
#         self.loop_rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             rospy.loginfo("nya")
#             cv2.imshow('img', self.cv_image)
#             rospy.loginfo("nyan")
#             self.loop_rate.sleep()
#             pass
            
# def main(args):
#     try:
#         rospy.init_node("jaguar_indicator")
#         func = jaguar_indicator()
#     except:
#         rospy.loginfo("jaguar_indicator : something wrong")
#     finally:
#         rospy.loginfo("jagura_indicator : process end")

        
# if __name__ == '__main__':
#     main(sys.argv)

# import rospy
# import cv2
# from cv2 import THRESH_BINARY
# import tf
# import geometry_msgs.msg
# import math
# import numpy as np
# from rospy.numpy_msg import numpy_msg
# from std_msgs.msg import String
# from std_msgs.msg import Float32MultiArray

# jaguar_hight = 0.088
# camera_angle = 55 #degree

# class jaguar_indicator:

#     def __init__(self,_channel,_loop_rate) -> None:
#         rospy.init_node("jaguar_indicator")
#         rospy.loginfo("init class")
#         self.capture = cv2.VideoCapture(_channel)
#         rospy.loginfo("capture image")
#         # self.locate_pub = rospy.Publisher("circle_location",numpy_msg(int),queue_size=100)
# #        self.tf_listener = tf.TransformListener()
#         rospy.loginfo("loaded tf")
#         self.capture.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
#         self.WIDTH = self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
#         self.r=rospy.Rate(_loop_rate)
#         rospy.loginfo("FLAG")
    
#         self.update()        
    
#     def getcircle(self):
#         self.ret,self.frame = self.capture.read()
#         if not self.ret:
#             rospy.loginfo("fail to get video image")
#             no_array = np.array([0,0,0])
#             return no_array
        
#         hsv_lower = np.array([30, 0, 0])
#         hsv_upper = np.array([90, 255 ,127])
        
#         hsv = cv2.cvtColor(self.frame,cv2.COLOR_BGR2HSV)
#         hsv_mask = cv2.inRange(hsv,hsv_lower,hsv_upper)
#         result = cv2.bitwise_and(frame,frame,mask=hsv_mask)     
#         dst_grey = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
#         retval,dst_bi = cv2.threshold(dst_grey,50,200,THRESH_BINARY)
# #        my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=0.084/self.pic_to_m,maxRadius=0.092/self.pic_to_m)
#         my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=75,maxRadius=200)

#         if not isinstance(my_circles,np.ndarray) :
#             rospy.loginfo("Circle is not found")
#             no_array = np.array([0,0,0])
#             return no_array
       
#         return my_circles
    
#     def update(self):
#         rospy.loginfo("enter main routine")
#         while not rospy.is_shutdown():
#             # (trans, rot) = self.tf_listener.lookupTransform('/base_link', '/camera', rospy.Time(0))
#             # self.pic_to_m = 2*(trans[2]-jaguar_hight) * math.tan(math.radians(camera_angle/2))/self.WIDTH
#             got_circle = np.array([0,0,0])
#             got_circle = self.getcircle()
#             # self.locate_pub.publish(got_circle)
#             rospy.loginfo("X : ",got_circle[0]," Y : ",got_circle[1]," R : ",got_circle[2])
#             cv2.imshow("nya",self.frame)
#             self.r.sleep()    
        
#         self.capture.release()


# if __name__ == "__main__" :
#     try:
#         camera_channel = rospy.get_param("~channel",1)
#         loop_rate = rospy.get_param("~loop_rate",20)
#         rospy.loginfo("program started!")
#         func = jaguar_indicator(camera_channel,loop_rate)
#     except:
#         rospy.loginfo("Error: something wrong")
#         pass
