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
        sub_image_topic_name = rosparam.get_param("theta_simple_stitching/sub_image_topic_name")
    except:
        rospy.logwarn("subscribe /image_raw because rosparam is not set")
        sub_image_topic_name = "/image_raw"

    self.image_pub = rospy.Publisher("/image/mercator", Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(sub_image_topic_name, Image, self.callback)


  def callback(self,data):
    try:
      image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    image_s = cv2.resize(image, (1280,720))

    vertex = 640
    src_cx = 319
    src_cy = 319
    src_r = 283
    src_cx2 = 1280 - src_cx

    map_x = np.zeros((vertex,vertex*2))
    map_y = np.zeros((vertex,vertex*2))
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
                map_x[y,x] = src_r * r_ * math.cos(theta2) + src_cx
                map_y[y,x] = src_r * r_ * math.sin(theta2) + src_cy
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
                map_x[y,x] = src_r * r_ * math.cos(math.pi - theta2) + src_cx2
                map_y[y,x] = src_r * r_ * math.sin(math.pi - theta2) + src_cy

    map_x = map_x.astype('float32')
    map_y = map_y.astype('float32')

    image2 = cv2.remap( image_s, map_x, map_y, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT);

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(image2, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main():
  ss = simple_stitching()
  rospy.init_node('theta_simple_stitching')
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")


if __name__ == '__main__':
    main()
