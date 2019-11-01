#!/usr/bin/env python
"""
Script to move to Map location - Assignment 3 Part 1
"""
import os
import rospy
import time
import getpass
import pandas as pd
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point



ACTION_DICT = {"1":"Go to Corner One","2":"Go to Corner Two","3":"Go to Corner Three","4":"Go to Corner Four","5":"Go Outside Door","0":"Quit"}


def get_goal():

	goal = ""
	# print "\n\nSelect a Goal\n"
	rospy.loginfo("\n\n Select a Goal")
	for key in ACTION_DICT:
		print "{0} :: {1}".format(key, ACTION_DICT[key])
		rospy.loginfo("{0} :: {1}".format(key, ACTION_DICT[key]))
	rospy.loginfo("\n\n")
	# print "\n\n"
	goal = str(input())
	# goal = input()
	# print "Typed:: ", goal
	# print "Data Typed:: ", type(goal)
	return goal


def go_to_goal(move_base, x, y, z, w):

	goal_sent = True
	goal_obj = MoveBaseGoal()
	goal_obj.target_pose.header.frame_id = 'map'
	goal_obj.target_pose.header.stamp = rospy.Time.now()
	goal_obj.target_pose.pose.position = Point(x, y, 0.000)

	goal_obj.target_pose.pose.orientation.x = 0.0
	goal_obj.target_pose.pose.orientation.y = 0.0
	goal_obj.target_pose.pose.orientation.z = z
	goal_obj.target_pose.pose.orientation.w = w

	rospy.loginfo("Moving To Goal Location")
	move_base.send_goal(goal_obj)
	success = move_base.wait_for_result(rospy.Duration(60))
	state = move_base.get_state()
	if(state ==  GoalStatus.SUCCEEDED and success):
		rospy.loginfo("Reached To Goal Location !!! :)")
		return True
	else:
		rospy.loginfo("Failed To Reach Goal Location !!! :(")
		return False


def get_goal_cordinates(goal, cordinates, without_dict=False):
	global ACTION_DICT
	# print "goalis :: ", goal
	if without_dict:
		cord_values = cordinates.loc[cordinates["lable"] == goal.split(" ")[-2].lower()+"_"+goal.split(" ")[-1].lower()].values
	else:
		cord_values = cordinates.loc[cordinates["lable"] == ACTION_DICT[goal].split(" ")[-2].lower()+"_"+ACTION_DICT[goal].split(" ")[-1].lower()].values
	if len(cord_values)>0:
		x, y, z, w = cord_values[0][1],cord_values[0][2],cord_values[0][3],cord_values[0][4]
	else:
		rospy.loginfo("Sorry Can't Read Coordinates")
		rospy.loginfo("Try Again")
		x, y, z, w = None,None,None,None
	return x, y, z, w


def touchdown(move_base, cordinates):

	goal = get_goal()
	if goal == "q":
		# print "Bye Bye"
		rospy.loginfo("Bye Bye")
	elif goal != "0" and goal in ACTION_DICT:
		print "Goal :: ", goal
		x, y, z, w = get_goal_cordinates(goal, cordinates)
		if x != None:
			result = go_to_goal(move_base, x, y, z, w)
			if not result:
				rospy.loginfo("Give Me Another Chance, I'll Make You Proud")
	else:
		rospy.loginfo("Try Again")

	return goal


def main():

	# cordinates = pd.read_csv("../res/assign3InitPos")
	try:
		# cordinates = pd.read_csv("/home/orcrist/catkin_ws/src/VishwakarmaS/res/assign3InitPos")
		cordinates = pd.read_csv("/home/{0}/catkin_ws/src/VishwakarmaS/res/assign3InitPos".format(getpass.getuser()))
		rospy.init_node('touchdown', anonymous=False)
		rospy.loginfo("Working DIR :: {0}".format(os.getcwd()))
		move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		while not rospy.is_shutdown():
			rospy.loginfo("Wait for Goal Selection Info")
			goal = touchdown(move_base, cordinates)
			if goal == "0":
				rospy.loginfo("Turning off Package Touchdown")
				break;
			rospy.sleep(1.0)
		move_base.cancel_goal()
		rospy.loginfo("Stoped")
	except rospy.ROSInterruptException as rx:
		print "Error {0}".format(rx)
		rospy.loginfo("Ctrl-C caught. Quitting")
	except IOError as ex:
		print "Error {0}".format(ex)
		rospy.loginfo("Error {0}".format(ex))
	return 0


if __name__ == '__main__':
	main()
