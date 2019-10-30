#!/usr/bin/env python
"""
Script to move to Map location with voice commands - Assignment 3 Part 2
"""
import rospy
import actionlib
import pandas as pd
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
import speech_recognition as sr
from geometry_msgs.msg import Point
from sound_play.libsoundplay import SoundClient


VOCAB_DICT = {"MOTION_CMD": ["Stop","Back","Forward","Backward","Left","Right","Rotate"],\
				"GOAL_CMD": ["Corner One","Corner Two","Corner Three","Corner Four","Outside Door","Quit"]}

class Obeyer:
	def __init__(self, cordinates):

		# self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		# rospy.on_shutdown(self.cleanup)
		self.voice = rospy.get_param("~voice", "voice_don_diphone")
		self.wavepath = "../catkin_ws/src/VishwakarmaS/res/"
		# Create the sound client object
		self.soundhandle = SoundClient()
		rospy.sleep(1)
		self.soundhandle.stopAll()
		# Announce that we are ready for input
		self.soundhandle.playWave(self.wavepath + "R2D2b.wav")
		rospy.sleep(1)
		self.soundhandle.say("Ready", self.voice)
		rospy.loginfo("Say one of the navigation commands...")
		# Subscribe to the recognizer output
		rospy.Subscriber('/recognizer/output', String, self.confirm_cmd)

	def confirm_cmd(self, msg):
		rospy.loginfo('You Said :: {0}'.format(msg.data))
		self.soundhandle.playWave(self.wavepath + "R2D2c.wav")
		rospy.sleep(1)

		return 0

	def move_base(self):
		pass

	def get_goal_location(self, goal_string):
		pass











def main():
	cordinates = pd.read_csv("../catkin_ws/src/VishwakarmaS/res/assign3InitPos")
	Obeyer(cordinates)
	return 0


if __name__ == '__main__':
	main()
