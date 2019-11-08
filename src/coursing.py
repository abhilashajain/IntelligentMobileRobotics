#!/usr/bin/env python
"""
Script to chase an object using astra camera sensors - Assignment 4
"""
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
from cv_bridge import CvBridge, CvBridgeError


# Define a function to show the image in an OpenCV Window
def show_image(img):
  cv2.imshow("Image Window", img)
  cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg, ):
  # log some info about the image topic
  rospy.loginfo(img_msg.header)

  # Try to convert the ROS Image message to a CV2 Image
  try:
      cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
  except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))

  # Flip the image 90deg
  cv_image = cv2.transpose(cv_image)
  cv_image = cv2.flip(cv_image,1)

  # Show the converted image
  show_image(cv_image)


def main():
    rospy.init_node('coursing', anonymous=False)
    bridge = CvBridge() # Initialize the CvBridge class
    callback_lambda = lambda x: image_callback(x,bridge)
    sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_lambda)
    cv2.namedWindow("Image Window", 1)
    while not rospy.is_shutdown():
      rospy.spin()
    return 0

if __name__ == '__main__':
    main()
