#!/usr/bin/env python
"""
Script to draw Initials with Turtlesim
"""
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import logging
import time
import os


LOG_PATH = "../catkin_ws/src/VishwakarmaS/others/"


def create_log():
	if not os.path.exists(LOG_PATH):
		os.makedirs(LOG_PATH)
	logging.basicConfig(filename="{0}wonderlust.log".format(LOG_PATH),level=logging.DEBUG)
	return 0












def main():
	create_log()
	logging.info("THIS IS VishwakarmaS Package Ros Log: Assign 2\n\n")

	rospy.init_node('sagar_turtlesim')
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	sub = rospy.Subscriber('/turtle1/pose', Pose, log_pos)
	move_msg = Twist()
	create_s(pub, sub, move_msg)
	create_v(pub, sub, move_msg)
	return 0

if __name__ == '__main__':
	main()
