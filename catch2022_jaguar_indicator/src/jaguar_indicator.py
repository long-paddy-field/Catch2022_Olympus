#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv2 import THRESH_BINARY
from std_msgs.msg import Float32MultiArray
import math

class Jaguar_Indicator:
    def __init__(self,cam_ch,loop_rate):
        rospy.loginfo("jaguar_indicator : called constructor")
        rospy.init_node("jaguar_indicator")
        self.pub_jaguar_position = rospy.Publisher("jaguar_position",Float32MultiArray,queue_size= 100)
        rospy.Subscriber("current_position",Float32MultiArray,self.current_position_callback,queue_size=100)
        rospy.Subscriber("current_angle",Float32MultiArray,self.current_angle_callback,queue_size=100)
        self.r = rospy.Rate(loop_rate)
        self.l1 = 0.6
        self.l2 = 0.3
        self.jaguar_position = Float32MultiArray()
        self.current_position = Float32MultiArray()
        self.current_angle = Float32MultiArray()
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
            rospy.loginfo("failed to read")
        hsv_lower = np.array([30, 0, 0])
        hsv_upper = np.array([90, 255 ,127])
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv_mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
        result = cv2.bitwise_and(frame,frame,mask=hsv_mask)
        dst_grey = cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        _,dst_bi = cv2.threshold(dst_grey,50,200,THRESH_BINARY)
        # cv2.imshow("edge",dst_bi)
        #my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=0.084/self.pic_to_m,maxRadius=0.092/self.pic_to_m)
        self.my_circles = cv2.HoughCircles(dst_bi,cv2.HOUGH_GRADIENT,1,80, param1=100,param2=30,minRadius=75,maxRadius=200)
                  
        if not isinstance(self.my_circles,np.ndarray):
            rospy.loginfo("Circle is not found")
            no_array = np.array([0,0,0])
            cv2.imshow("img",frame)
            # cv2.imshow("bin",dst_bi)
            return no_array
        
        # cv2.circle(frame,(200,200),10,(255,0,0),2) #検出された円を一個ずつ参照できる
        # for i in self.my_circles[0,:]: #:で行を参照してる
            # draw the outer circle
            # cv2.circle(frame,(i[0],i[1]),i[2],(0,255,0),2)
            # rospy.loginfo("nya")
            # draw the center of the circle
            # cv2.circle(frame,(i[0],i[1]),2,(0,0,255),3)

        cv2.imshow("img", frame)

        return self.my_circles
    
    # def pixel_to_meter(self):
    #     self.meter_circles = 0.01 * self.my_circles
        
    #     # for i in self.my_circles[0,:]:
    #     #     self.pixel_x = i[0]
    #     #     self.pixel_y = i[1]
    #     #     self.pixel_radius = i[2]
    #     # #実機で検証するところ
    #     # self.meter_x = self.pixel_x
    #     # self.meter_y = self.pixel_y
    #     # self.meter_radius = self.pixel_radius 
    #     # meter = list([self.meter_x,self.meter_y,self.meter_radius])
    #     return self.meter_circles
    def current_position_callback(self,msg):
        self.cam_world_x = msg.data[0]#0.3#ロボット座標からみたcamの座標
        self.cam_world_y = msg.data[1]#0.3


    def current_angle_callback(self,msg):
        self.current_angle = msg.data
    

    def cal_jaguar_pos(self):
        if isinstance(self.my_circles,np.ndarray) :
            # rospy.loginfo("Circle is not found")
            # no_array = np.array([0,0,0])
            # return no_array
            self.meter_circles = 0.01 * (self.my_circles)
        # self.meter_circles =[1.2,0.7,0.5]
            
            for circle in self.meter_circles[0,:]:
                self.edge_jaguar_x = circle[0]#camの左上を原点としたjaguarの座標
                self.edge_jaguar_y = circle[1]
        
            #camの左上を原点としたcamの中心の座標
                self.edge_cam_x = 0.96#1*960 #pixel単位からm単位に
                self.edge_cam_y = 0.54#1*540

                self.cam_jaguar_x = self.edge_jaguar_x - self.edge_cam_x 
                self.cam_jaguar_y = -(self.edge_jaguar_y - self.edge_cam_y)
                #ジャガのロボット座標への座標変換
                # self.jaguar_x = self.cam_world_x+(self.cam_jaguar_x*(math.sin(math.radians(60+50)))+self.cam_jaguar_y*(math.cos(math.radians(60+50))))
                # self.jaguar_y = self.cam_world_y+(self.cam_jaguar_y*(math.sin(math.radians(60+50)))-self.cam_jaguar_x*(math.cos(math.radians(60+50))))
                self.jaguar_x = self.cam_world_x+(self.cam_jaguar_x*(math.sin(self.current_angle.data[0]+self.current_angle[1]))+self.cam_jaguar_y*(math.cos(self.current_angle.data[0]+self.current_angle[1])))
                self.jaguar_y = self.cam_world_y+(self.cam_jaguar_y*(math.sin(self.current_angle.data[0]+self.current_angle[1]))-self.cam_jaguar_x*(math.cos(self.current_angle.data[0]+self.current_angle[1])))
                self.jaguar_position.data = [self.jaguar_x,self.jaguar_y]
            # for i in self.meter_list[0,:]:
            #     self.jaguar_position_x = (i[0]*(math.sin(self.current_angle.data[0]+self.current_angle[1]))+i[1]*(math.cos(self.current_angle.data[0]+self.current_angle[1])))+self.current_position.data[0]
            #     self.jaguar_position_y = (i[0]*(math.cos(self.current_angle.data[0]+self.current_angle[1]))+i[1]*(math.sin(self.current_angle.data[0]+self.current_angle[1])))+self.current_position.data[1]
            #     self.jaguar_position = np.array([self.jaguar_position_x,self.jaguar_position_y])
                rospy.loginfo(self.jaguar_position.data)
                self.pub_jaguar_position.publish(self.jaguar_position)

    def update(self):
        self.got_circle = list()
        while not rospy.is_shutdown():
            self.got_circle = self.getcircle()
            self.cal_jaguar_pos()
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rospy.loginfo(self.got_circle)
            self.r.sleep()

if __name__=='__main__':
    # try:
    func = Jaguar_Indicator(cam_ch=4,loop_rate=50) #cam_ch 4
    # except:
    #     rospy.logwarn("jaguar_indicator : something wrong")
    # finally:
    #     rospy.logwarn("jaguar_indicator : end process")

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