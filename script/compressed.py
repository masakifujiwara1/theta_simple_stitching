#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image/mercator", Image, self.image_callback)
        self.compressed_image_pub = rospy.Publisher("/image/mercator/compressed", CompressedImage, queue_size=10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            compressed_img = cv2.imencode('.jpg', cv_image)[1].tostring()

            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = compressed_img

            self.compressed_image_pub.publish(compressed_msg)

        except Exception as e:
            rospy.logerr("Image conversion failed: %s" % e)

def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()
    rospy.wait_for_message("/image/mercator", Image)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()

