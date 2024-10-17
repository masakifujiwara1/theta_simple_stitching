#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
import rosparam
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class simple_stitching:
  def __init__(self):
    try:
        sub_image1_topic_name = rosparam.get_param("theta_simple_stitching/sub_image1_topic_name")
        sub_image2_topic_name = rosparam.get_param("theta_simple_stitching/sub_image2_topic_name")
    except:
        rospy.logwarn("subscribe /image_raw because rosparam is not set")
        sub_image1_topic_name = "/image1_raw"
        sub_image2_topic_name = "/image2_raw"
    try:
        self.reverse = rosparam.get_param("theta_simple_stitching/reverse")
    except:
        rospy.logwarn("image is not reversed")
        self.reverse = False

    self.image_pub = rospy.Publisher("/image/mercator", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image1_sub = rospy.Subscriber(sub_image1_topic_name, Image, self.image1_callback)
    self.image2_sub = rospy.Subscriber(sub_image2_topic_name, Image, self.image2_callback)
    self.image_s1 = np.array
    self.image_s2 = np.array

    vertex = 640
    src_cx = 319
    src_cy = 319
    src_r = 319 #283
    src_cx2 = 1280 - src_cx

    self.map_x = np.zeros((vertex,vertex*2))
    self.map_y = np.zeros((vertex,vertex*2))
    for y in range(vertex):
        for x in range(vertex*2):
            phi1 = math.pi * x / vertex
            theta1 = math.pi * y / vertex

            X = math.sin(theta1) * math.cos(phi1)
            Y = math.sin(theta1) * math.sin(phi1)
            Z = math.cos(theta1)

            phi2 = math.acos(-X)
            theta2 = np.sign(Y)*math.acos(-Z/math.sqrt(Y*Y + Z*Z))

            #0 equidistant projection
            #1 stereographic projection
            #2 stereographic inverse projection
            #3 Orthogonal projection
            #4 Orthogonal inverse projection
            method = 0

            if(phi2 < math.pi / 2):
                if method == 0:
                    r_ = phi2 / math.pi * 2
                elif method == 1:
                    r_ =  math.tan((phi2) / 2)
                elif method == 2:
                    r_ =  1 - math.tan((math.pi / 2 - phi2) / 2)
                elif method == 3:
                    r_ = math.sin(phi2)
                elif method == 4:
                    r_ = 1 - math.sin(math.pi / 2 - phi2)
                self.map_x[y,x] = src_r * r_ * math.cos(theta2) + src_cx
                self.map_y[y,x] = src_r * r_ * math.sin(theta2) + src_cy
            else:
                if method == 0:
                    r_ = (math.pi - phi2) / math.pi * 2
                elif method == 1:
                    r_ =  math.tan((math.pi - phi2) / 2)
                elif method == 2:
                    r_ =  1 - math.tan((-math.pi/2 + phi2) / 2)
                elif method == 3:
                    r_ = math.sin(math.pi - phi2)
                elif method == 4:
                    r_ = 1 - math.sin(- math.pi / 2 + phi2)
                self.map_x[y,x] = src_r * r_ * math.cos(math.pi - theta2) + src_cx2
                self.map_y[y,x] = src_r * r_ * math.sin(math.pi - theta2) + src_cy

    self.map_x = self.map_x.astype('float32')
    self.map_y = self.map_y.astype('float32')
    print("finish init for theta simple stitching")


  def image1_callback(self,data):
    try:
      image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.image_s1 = cv2.resize(image1, (640,640))

  def image2_callback(self,data):
    try:
      image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    self.image_s2 = cv2.resize(image2, (640,640))
    
  def stitching(self):
    image_cat = cv2.hconcat([self.image_s1, self.image_s2])
    image_remap = cv2.remap( image_cat, self.map_x, self.map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT);
    if(self.reverse):
        tmp = image_remap.copy()
        image_remap[:, :1280/2,:] = tmp[:, 1280/2:,:]
        image_remap[:, 1280/2:,:] = tmp[:, :1280/2,:]

    msg_for_send = self.bridge.cv2_to_imgmsg(image_remap, "bgr8")
    # msg_for_send.header = Image.header

    try:
      self.image_pub.publish(msg_for_send)
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':
    rospy.init_node('theta_simple_stitching')
    ss = simple_stitching()
    rospy.wait_for_message("/camera/rgb/image_raw_front", Image)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        ss.stitching()
        rate.sleep()
        
