#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

class CompressedImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/theta/image_raw/compressed", CompressedImage, self.callback)
        self.image_pub = rospy.Publisher("/theta/image_raw", Image, queue_size=1)

    def callback(self, data):
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            
            self.image_pub.publish(ros_image)

        except Exception as e:
            rospy.logerr("Failed to convert CompressedImage to Image: %s" % str(e))

def main():
    rospy.init_node('compressed_image_converter', anonymous=True)
    converter = CompressedImageConverter()
    rospy.wait_for_message("/theta/image_raw/compressed", CompressedImage)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
