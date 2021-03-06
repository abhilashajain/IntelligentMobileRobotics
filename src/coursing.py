#!/usr/bin/env python
"""
Script to chase an object using astra camera sensors - Assignment 4
"""
# Import ROS libraries and messages
from __future__ import print_function
import rospy
from sensor_msgs.msg import Image
from collections import deque
from geometry_msgs.msg import Twist
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

LOWER_BLUE = (110,50,50)
UPPER_BLUE = (130,255,255)

POINTS = deque(maxlen=32)
COUNTER = 0



def draw_hough_circle(dilated_mask):

	detected_circles = cv2.HoughCircles(dilated_mask, cv2.HOUGH_GRADIENT, 1, 150, param1=100, param2=20, minRadius=10, maxRadius=20)

	if detected_circles is not None:
		for circle in detected_circles[0, :]:
			circled_orig = cv2.circle(frame, (circle[0], circle[1]), circle[2], (0,255,0),thickness=3)
		return True, circled_orig
	else:
		return False, dilprocess_rgb_frameated_mask


def process_rgb_frame(frame, colour):
	# blusing the image to reduce noise
	blurred_frame = cv2.GaussianBlur(frame, (9, 9),3,3)
	blurred_frame = cv2.GaussianBlur(blurred_frame, (9, 9),3,3)
	# converting the input stream into HSV color space
	blurred_hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
	# blurred_hsv_frame = cv2.GaussianBlur(hsv_frame,(9,9),3,3)
	if colour == "red":
		red_mask_0 = cv2.inRange(blurred_hsv_frame, RED_LOWER_BOUNDS_0, RED_UPPER_BOUNDS_0)
		red_mask_1 = cv2.inRange(blurred_hsv_frame, RED_LOWER_BOUNDS_1, RED_UPPER_BOUNDS_1)
		# after masking the red shades out, I add the two images
		weighted_mask = cv2.addWeighted(red_mask_0, 1.0, red_mask_1, 1.0, 0.0)
		# some morphological operations (closing) to remove small blobs
	if colour == "blue":
		weighted_mask = cv2.inRange(blurred_hsv_frame, LOWER_BLUE, UPPER_BLUE)
	erode_element = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
	dilate_element = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8))
	eroded_mask = cv2.erode(weighted_mask,erode_element)
	dilated_mask = cv2.dilate(eroded_mask,dilate_element)

	return dilated_mask


def compute_the_contour(dilated_mask, frame):
	global POINTS
	# (x, y) center of the ball
	cnts = cv2.findContours(dilated_mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
	# update the points queue
	POINTS.appendleft(center)
	# loop over the set of tracked points
	for i in range(1, len(POINTS)):
		# if either of the tracked points are None, ignore
		if POINTS[i - 1] is None or POINTS[i] is None:
			continue
		# otherwise, compute the thickness of the line and draw the connecting lines
		cv2.line(frame, POINTS[i - 1], POINTS[i], (0, 0, 255), 3)

	return frame


def opencv_test():
	# read image into rgb frame
	frame = cv2.imread('../others/aster_red.jpg')
	frame = cv2.resize(frame,(640,480))
	dilated_mask = process_rgb_frame(frame, "blue")
	# flag, circled_orig = draw_hough_circle(dilated_mask)
	frame = compute_the_contour(dilated_mask, frame)
	while True:
		cv2.imshow("frames",frame)
		if cv2.waitKey(1)&0xFF == ord('q'):
			break;
	return 0

def get_motion_command(direction):
	move_msg = Twist()

	if direction == "left":
		move_msg.linear.x = 0.1
		move_msg.angular.z = 0.15
	elif direction == "right":
		move_msg.linear.x = 0.1
		move_msg.angular.z = -0.15
	elif direction == "forward":
		move_msg.linear.x = 0.2
		move_msg.angular.z = 0
	elif direction == "back":
		move_msg.linear.x = -0.2
		move_msg.angular.z = 0
	elif direction == "forward-right":
		move_msg.linear.x = 0.2
		move_msg.angular.z = -0.15
	elif direction == "forward-left":
		move_msg.linear.x = 0.2
		move_msg.angular.z = 0.15
	elif direction == "back-right":
		move_msg.linear.x = -0.2
		move_msg.angular.z = -0.15
	elif direction == "back-left":
		move_msg.linear.x = -0.2
		move_msg.angular.z = 0.15
	else:
		move_msg.linear.x = 0.1
		move_msg.angular.z = 0
		# print("Not Moving\n")
		return move_msg, False

	return move_msg, True


def track_movement(frame):
	direction = ""
	# print(POINTS)
	# loop over the set of tracked points
	for i in np.arange(1, len(POINTS)):
		# if either of the tracked points are None, ignore
		if POINTS[i - 1] is None or POINTS[i] is None:
			continue
		# check to see if enough points have been accumulated in the buffer
		if COUNTER >= 20 and i == 1 and POINTS[-10] is not None:
			# compute the difference between the x and y coordinates and re-initialize the direction
			dX = POINTS[-10][0] - POINTS[i][0]
			dY = POINTS[-10][1] - POINTS[i][1]
			dirX, dirY = "", ""
			# ensure there is significant movement in the x-direction
			if np.abs(dX) > 20:
				dirX = "right" if np.sign(dX) == 1 else "left"
			# ensure there is significant movement in the y-direction
			if np.abs(dY) > 20:
				dirY = "forward" if np.sign(dY) == 1 else "back"
			# handle when both directions are non-empty
			if dirX != "" and dirY != "":
				direction = "{}-{}".format(dirY, dirX)
			# otherwise, only one direction is non-empty
			elif dirX == "" and dirY != "":
				direction = dirY
			elif dirY == "" and dirX != "":
				direction = dirX
			else:
				direction = "nowhere"
	# cv2.putText(frame, direction, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 3)
	return direction


# Define a callback for the Image message
def image_callback(img_msg, bridge, motion_pub):
	global COUNTER
	# rospy.loginfo(img_msg.header)
	COUNTER = COUNTER + 1
	# Try to convert the ROS Image message to a CV2 Image
	try:
		frame = bridge.imgmsg_to_cv2(img_msg, "passthrough")
	except CvBridgeError, e:
		rospy.logerr("CvBridge Error: {0}".format(e))

	dilated_mask = process_rgb_frame(frame, "blue")
	# flag, circled_orig = draw_hough_circle(dilated_mask)
	frame = compute_the_contour(dilated_mask, frame)
	direction = track_movement(frame)
	move_msg, move_flag = get_motion_command(direction)
	if move_flag:
		COUNTER = 0
		motion_pub.publish(move_msg)
	else:
		pass
	if COUNTER >= 1000:
		COUNTER = 0
	cv2.imshow('img', frame)
	if cv2.waitKey(1) & 0xFF == ord("q"):
		exit(0)
	return 0


def run_ros():
	rospy.init_node('coursing', anonymous=False)
	motion_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)
	bridge = CvBridge() # Initialize the CvBridge class
	callback_lambda = lambda x: image_callback(x, bridge, motion_pub)
	sub_image = rospy.Subscriber("/camera/rgb/image_raw", Image, callback_lambda, queue_size=1)
	while not rospy.is_shutdown():
	  rospy.spin()
	  # time.sleep(1)
	return 0


def main():
	# opencv_test()
	run_ros()
	return 0


if __name__ == '__main__':
	main()
