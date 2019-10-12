#!/usr/bin/env python
"""
Script to draw Map with Turtlesim - Assignment 2
"""
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import logging
from random import seed
import random
import time
import os


seed(4)

LOG_PATH = "../catkin_ws/src/VishwakarmaS/others/"

LASER_REGIONS = {"right":0,"fright":0,"front":0,"fleft":0,"left":0}
TURTLEBOT_STATE = 0
TURTLEBOT_TAKE_TURN = "nothing"
TURTLEBOT_CRASHED =  False
FOND_A_DOOR = False
PREVIOUS_TURN_NEG = False
DISTANCE = 1.0
TURTLEBOT_STATE_DICT = {0:"find the wall",1:"turn left",2:"follow the wall", 3:"stuck"}
TURTLEBOT_PREV_STATE = {"prev_state":0,"count":0}


def create_log():
	if not os.path.exists(LOG_PATH):
		os.makedirs(LOG_PATH)
	logging.basicConfig(filename="{0}wonderlust.log".format(LOG_PATH),level=logging.DEBUG)
	return 0


def go_out_of_unknown():
	# need to handle the unknown state to chnage state not randombly but after few iteration opposit to PREVIOUS
	move_msg = Twist()
	# move_msg.angular.z = -0.3
	move_msg.angular.z = random.uniform(-1, 1)/3
	# move_msg.linear.x = 0.2
	move_msg.linear.x = random.uniform(0, 1)/3
	return move_msg


def go_back():
	global TURTLEBOT_CRASHED, PREVIOUS_TURN_NEG
	# need to handle the unknown state to chnage state not randombly but after few iteration opposit to PREVIOUS
	move_msg = Twist()
	if TURTLEBOT_CRASHED:
		move_msg.linear.x = -0.8
		angular_z = random.uniform(-1, 1)/2
	else:
		angular_z = random.uniform(-1, 1)/3
		angular_z = -0.3
	if PREVIOUS_TURN_NEG:
		move_msg.angular.z = abs(angular_z)
		PREVIOUS_TURN_NEG = False
	else:
		move_msg.angular.z = -angular_z
		PREVIOUS_TURN_NEG = True
	return move_msg


def take_turn():
	global TURTLEBOT_TAKE_TURN
	move_msg = Twist()
	if TURTLEBOT_TAKE_TURN == "right":
		move_msg.angular.z = -0.3
	elif TURTLEBOT_TAKE_TURN == "left":
		move_msg.angular.z = 0.3
	else:
		move_msg.angular.z = 0.0
	return move_msg


def follow_the_wall():
	move_msg = take_turn()
	if FOND_A_DOOR:
		move_msg.linear.x = 0.4
	else:
		move_msg.linear.x = 0.25
	return move_msg


def change_state(state_value):
	global TURTLEBOT_STATE
	TURTLEBOT_STATE = state_value
	return 0


def take_action():
	global DISTANCE, LASER_REGIONS, TURTLEBOT_TAKE_TURN, TURTLEBOT_CRASHED, FOND_A_DOOR

	if LASER_REGIONS['front'] > DISTANCE and LASER_REGIONS['fleft'] > DISTANCE and LASER_REGIONS['fright'] > DISTANCE:
		state_description = "Case 1 :: Nothing, State 0"
		TURTLEBOT_TAKE_TURN = "nothing"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = True
		change_state(0)
	elif LASER_REGIONS['front'] < DISTANCE and LASER_REGIONS['fleft'] > DISTANCE and LASER_REGIONS['fright'] > DISTANCE:
		# take small left or right and dont move
		state_description = "Case 2 :: front, State 1"
		if LASER_REGIONS['fleft'] > LASER_REGIONS['fright']:
			TURTLEBOT_TAKE_TURN = "left"
		else:
			TURTLEBOT_TAKE_TURN = "right"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = False
		change_state(1)
	elif LASER_REGIONS['front'] > DISTANCE and LASER_REGIONS['fleft'] > DISTANCE and LASER_REGIONS['fright'] < DISTANCE:
		# take small left and move
		state_description = "Case 3 :: fright, State 2"
		TURTLEBOT_TAKE_TURN = "left"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = False
		change_state(2)
	elif LASER_REGIONS['front'] > DISTANCE and LASER_REGIONS['fleft'] < DISTANCE and LASER_REGIONS['fright'] > DISTANCE:
		# take small right and move
		state_description = "Case 4 :: fleft, State 0"
		TURTLEBOT_TAKE_TURN = "right"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = False
		change_state(0)
	elif LASER_REGIONS['front'] < DISTANCE and LASER_REGIONS['fleft'] > DISTANCE and LASER_REGIONS['fright'] < DISTANCE:
		# take little big left and dont move
		state_description = "Case 5 :: front - fright, State 1"
		TURTLEBOT_TAKE_TURN = "left"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = False
		change_state(1)
	elif LASER_REGIONS['front'] < DISTANCE and LASER_REGIONS['fleft'] < DISTANCE and LASER_REGIONS['fright'] > DISTANCE:
		# take little big right and dont move
		state_description = "Case 6 :: front - fleft, State 1"
		TURTLEBOT_TAKE_TURN = "right"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = False
		change_state(1)
	elif LASER_REGIONS['front'] < DISTANCE and LASER_REGIONS['fleft'] < DISTANCE and LASER_REGIONS['fright'] < DISTANCE:
		# hit a wall go back take left or right
		state_description = "Case 7 :: front - fleft - fright, State 3"
		if LASER_REGIONS['fleft'] < LASER_REGIONS['fright']:
			TURTLEBOT_TAKE_TURN = "right"
		else:
			TURTLEBOT_TAKE_TURN = "left"
		TURTLEBOT_CRASHED =  True
		FOND_A_DOOR = False
		change_state(3)
	elif LASER_REGIONS['front'] > DISTANCE and LASER_REGIONS['fleft'] < DISTANCE and LASER_REGIONS['fright'] < DISTANCE:
		# got front without and turn
		state_description = "Case 8 :: fleft - fright, State 0"
		TURTLEBOT_TAKE_TURN = "nothing"
		TURTLEBOT_CRASHED =  False
		FOND_A_DOOR = True
		change_state(0)
	else:
		state_description = "Unknown Case"
		change_state(4)
		# rospy.loginfo(LASER_REGIONS)
	rospy.loginfo(state_description)
	# print(state_description)
	return 0


def update_variables(laser_range_list):
	global LASER_REGIONS

	right_nan_count = sum(np.isnan(laser_range_list[0]))
	fright_nan_count = sum(np.isnan(laser_range_list[1]))
	front_nan_count = sum(np.isnan(laser_range_list[2]))
	fleft_nan_count = sum(np.isnan(laser_range_list[3]))
	left_nan_count = sum(np.isnan(laser_range_list[4]))

	if right_nan_count > laser_range_list[0].size*0.8:
		LASER_REGIONS["right"] = 0
	else:
		LASER_REGIONS["right"] = min(laser_range_list[0][np.isfinite(laser_range_list[0])].min(), 10)

	if fright_nan_count > laser_range_list[1].size*0.8:
		LASER_REGIONS["fright"] = 0
	else:
		LASER_REGIONS["fright"] = min(laser_range_list[1][np.isfinite(laser_range_list[1])].min(), 10)

	if front_nan_count > laser_range_list[2].size*0.8:
		LASER_REGIONS["front"] = 0
	else:
		LASER_REGIONS["front"] = min(laser_range_list[2][np.isfinite(laser_range_list[2])].min(), 10)

	if fleft_nan_count > laser_range_list[3].size*0.8:
		LASER_REGIONS["fleft"] = 0
	else:
		LASER_REGIONS["fleft"] = min(laser_range_list[3][np.isfinite(laser_range_list[3])].min(), 10)

	if left_nan_count > laser_range_list[3].size*0.8:
		LASER_REGIONS["left"] = 0
	else:
		LASER_REGIONS["left"] = min(laser_range_list[4][np.isfinite(laser_range_list[4])].min(), 10)

	return 0


def read_lasers(sub_msg):

	laser_range = np.array(list(sub_msg.ranges))

	laser_range_list = np.split(laser_range, 5)

	update_variables(laser_range_list)

	take_action()
	return 0


def move_turtlebot(pub_command, pub_command_control, sub):
	global TURTLEBOT_STATE
	# time.sleep(5)
	recieved_msg = None
	rate = rospy.Rate(20)
	while not rospy.is_shutdown():
		move_msg = Twist()
		if TURTLEBOT_STATE == 0:
			move_msg = follow_the_wall()
			pub_command.publish(move_msg)
		elif TURTLEBOT_STATE == 1:
			move_msg = take_turn()
			pub_command.publish(move_msg)
		elif TURTLEBOT_STATE == 2:
			move_msg = follow_the_wall()
			pub_command.publish(move_msg)
		elif TURTLEBOT_STATE == 3:
			move_msg = go_back()
			pub_command.publish(move_msg)
		else:
			move_msg = go_out_of_unknown()
			rospy.loginfo("Unknown State")
			pub_command.publish(move_msg)
		time.sleep(1)
		rate.sleep()
	return 0


def main():
	# create_log()
	# logging.info("THIS IS VishwakarmaS Package Ros Log: Assign 2\n\n")
	rospy.init_node("wonderlust", anonymous=False)
	pub_command = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size = 10)
	pub_command_control = rospy.Publisher('cmd_vel_mux/input/safety_controller', Twist, queue_size = 1)
	sub = rospy.Subscriber('scan', LaserScan, read_lasers)
	move_msg = Twist()
	move_msg.angular.z = -1.0
	pub_command.publish(move_msg)
	time.sleep(1)
	move_turtlebot(pub_command, pub_command_control, sub)
	return 0

if __name__ == '__main__':
	main()
