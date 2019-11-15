#!/usr/bin/env python
"""
Script to chase an object using astra camera sensors - Assignment 4
"""
# Import ROS libraries and messages
import rospy
from sensor_msgs.msg import Image

# Import OpenCV libraries and tools
import cv2
import numpy as np
import time
import imutils
from cv_bridge import CvBridge, CvBridgeError



RED_LOWER_BOUNDS_0 = (0, 100, 100)
RED_UPPER_BOUNDS_0 = (10, 255, 255)
RED_LOWER_BOUNDS_1 = (160, 100, 100)
RED_UPPER_BOUNDS_1 = (180, 255, 255)

CIRCLE_RADIUS = 2

# Define a function to show the image in an OpenCV Window
def show_image(img):
  cv2.imshow("Image Window", img)
  cv2.waitKey(3)

# Define a callback for the Image message
def image_callback(img_msg, bridge):
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
  print "IMAGE DATA \n"
  print cv_image, "\n"
  time.sleep(2)

  # Show the converted image
  # show_image(cv_image)

def draw_circle(frame):
	# blusing the image to reduce noise
	blurred_frame = cv2.GaussianBlur(frame, (9, 9),3,3)
	# converting the input stream into HSV color space
	blurred_hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
	# blurred_hsv_frame = cv2.GaussianBlur(hsv_frame,(9,9),3,3)
	red_mask_0 = cv2.inRange(blurred_hsv_frame, RED_LOWER_BOUNDS_0, RED_UPPER_BOUNDS_0)
	red_mask_1 = cv2.inRange(blurred_hsv_frame, RED_LOWER_BOUNDS_1, RED_UPPER_BOUNDS_1)
	# after masking the red shades out, I add the two images
	weighted_mask = cv2.addWeighted(red_mask_0, 1.0, red_mask_1, 1.0, 0.0)

	# some morphological operations (closing) to remove small blobs
	erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))

	eroded_mask = cv2.erode(weighted_mask,erode_element)
	dilated_mask = cv2.dilate(eroded_mask,dilate_element)

	# red_mask = cv2.erode(red_mask, None, iterations=2)
	# red_mask = cv2.dilate(red_mask, None, iterations=2)
	detected_circles = cv2.HoughCircles(dilated_mask, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=10, maxRadius=200)

	if detected_circles is not None:
		for circle in detected_circles[0, :]:
			circled_orig = cv2.circle(frame, (circle[0], circle[1]), circle[2], (0,255,0),thickness=3)
		return circled_orig
	else:
		return frame


def red_circle_image():
	# read image into rgb frame
	frame = cv2.imread('./res/red_circle.jpg')
	frame_ = draw_circle(frame)
	cnts = cv2.findContours(red_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None


def opencv_test():
	cap = cv2.VideoCapture(0)




	# some morphological operations (closing) to remove small blobs
	# erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	# dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
	# eroded_mask = cv2.erode(red_mask,erode_element)
	# dilated_mask = cv2.dilate(eroded_mask,dilate_element)

	# on the color-masked, blurred and morphed image I apply the cv2.HoughCircles-method to detect circle-shaped objects
	# detected_circles = cv2.HoughCircles(dilated_mask, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=0, maxRadius=200)




	#
	# bright_red_lower_bounds = (0, 100, 100)
	# bright_red_upper_bounds = (10, 255, 255)
	#
	# v_frame, s_frame, h_frame  = cv2.split(hsv_frame)
	#
	# hsv_conv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	#
	# # ret, h_frame_thresh_abv = cv2.threshold(h_frame, 70, 255, cv2.THRESH_BINARY_INV)
	# # ret, h_frame_thresh_abv = cv2.threshold(h_frame, 70, 255, cv2.THRESH_BINARY)
	#
	# # h_frame_thresh_abv = cv2.inRange(h_frame, 60, 70)
	# # s_frame_thresh_abv = cv2.inRange(s_frame, 60, 70)
	# v_frame_thresh_abv = cv2.inRange(v_frame, 60, 70)
	#

	# cv2.imwrite('./res/red_circle_h.jpg', h_frame)
	# cv2.imwrite('./res/red_circle_s.jpg', s_frame)
	# cv2.imwrite('./res/red_circle_v.jpg', v_frame)

	while(True):
		# Capture frame-by-frame
		ret, frame = cap.read()


		frame_ = draw_circle(frame)
		# cv2.imshow('frame',dark_red_mask)
		# cv2.imshow('frame',s_frame)
		# cv2.imshow('frame',h_frame_thresh_abv)
		# cv2.imshow('frame',s_frame_thresh_abv)
		# cv2.imshow('frame',v_frame_thresh_abv)
		cv2.imshow('frame',frame_)


		# Our operations on the frame come here
		# gray = cv2.transpose(gray)
		# gray = cv2.flip(gray,1)

		# Display the resulting frame
		# print "IMAGE\n"
		# print frame.shape
		# print gray,"\n"
		# time.sleep(0.5)
		# cv2.imshow('frame',gray)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	# When everything done, release the capture
	# cap.release()
	cv2.destroyAllWindows()



def run_ros():
	rospy.init_node('coursing', anonymous=False)
	bridge = CvBridge() # Initialize the CvBridge class
	callback_lambda = lambda x: image_callback(x,bridge)
	sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_lambda, queue_size=1)
	# cv2.namedWindow("Image Window", 1)
	while not rospy.is_shutdown():
	  rospy.spin()
	  time.sleep(1)
	return 0


def main():
	opencv_test()
	return 0


if __name__ == '__main__':
	main()
