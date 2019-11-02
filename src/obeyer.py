#!/usr/bin/env python
"""
Script to move to Map location with voice commands - Assignment 3 Part 2
"""
import rospy
import sys
import getpass
import actionlib
import pandas as pd
from difflib import SequenceMatcher
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import *
import speech_recognition as sr
from sound_play.libsoundplay import SoundClient
import time

sys.path.insert(1, "/home/{0}/catkin_ws/src/VishwakarmaS/src/".format(getpass.getuser()))

import touchdown as td
import ros_voice_control as rvc


VOCAB_DICT = {"MOTION_CMD": ["stop","back","forward","backward","left","right","rotate"],\
				"GOAL_CMD": ["corner one","corner two","corner three","corner four","outside door","quit"],\
				"DECISION_CMD":["yes", "no"]}


def recognize_speech_from_mic(recognizer, microphone):
	start_time = time.time()
	with microphone as source:
		print "recording"
		recognizer.adjust_for_ambient_noise(source, duration = 2.0)
		# recognizer.record(source)
		print "listning"
		audio = recognizer.listen(source)
	print "Audio Capturing --- %s seconds ---" % (time.time() - start_time)
	response = {
		"success": True,
		"error": None,
		"transcription": None
	}
	try:
		start_time = time.time()
		response["transcription"] = recognizer.recognize_google(audio)
		# response["transcription"] = recognizer.recognize_sphinx(audio)
		print "Audio Recognize --- %s seconds ---" % (time.time() - start_time)
	except sr.RequestError:
		response["success"] = False
		response["error"] = "API unavailable"
	except sr.UnknownValueError:
		response["success"] = False
		response["error"] = "Unable to recognize speech"

	return response


def generic_motion(seq_cmd, move_msg, soundhandle, wavepath):

	if seq_cmd=="forward":
		move_msg.linear.x = 0.5
		move_msg.angular.z = 0.0
	elif seq_cmd=="backward":
		move_msg.linear.x = -0.5
		move_msg.angular.z = 0.0
	elif seq_cmd=="stop":
		move_msg.linear.x = 0.0
		move_msg.angular.z = 0.0
	elif seq_cmd=="back":
		move_msg.linear.x = -0.5
		move_msg.angular.z = 0.0
	elif seq_cmd=="left":
		move_msg.angular.z = 0.10
	elif seq_cmd=="right":
		move_msg.angular.z = -0.10
	elif seq_cmd=="rotate":
		move_msg.linear.x = 0.0
		move_msg.angular.z = 1
	else:
		soundhandle.playWave(wavepath + "R2D2b.wav")
		print "Sorry No Motion Selected"
	return True, seq_cmd, move_msg


def goal_motion(seq_cmd, move_base, cordinates, soundhandle, wavepath):

	if seq_cmd !="quit":
		x, y, z, w = td.get_goal_cordinates(seq_cmd, cordinates, True)
		if x != None:
			soundhandle.playWave(wavepath + "R2D2a.wav")
			result = td.go_to_goal(move_base, x, y, z, w)
			seq_cmd = "" # [None, None, 0, None]
			if not result:
				soundhandle.playWave(wavepath + "R2D2c.wav")
				rospy.loginfo("Give Me Another Chance, I'll Make You Proud")
	else:
		soundhandle.playWave(wavepath + "R2D2b.wav")
		return False, ""
	return True, seq_cmd


def match_responce(response, seq_cmd):
	cmd_found = False
	for cmd in VOCAB_DICT["MOTION_CMD"]:
		ratio = SequenceMatcher(None, cmd, response["transcription"]).ratio()
		if ratio > 0.7:
			cmd_found = True
			seq_cmd = cmd.lower()
	if not cmd_found:
		for cmd in VOCAB_DICT["GOAL_CMD"]:
			ratio = SequenceMatcher(None, cmd, response["transcription"]).ratio() # "Go to Corner One", "forner One" ratio 0.699
			if ratio > 0.7:
				cmd_found = True
				seq_cmd = cmd.lower()
		if not cmd_found:
			print "Can You be more clear next time !"
	return seq_cmd


def obeyer(cordinates, wavepath):
	soundhandle = SoundClient()
	recognizer = sr.Recognizer()
	microphone = sr.Microphone()
	move_msg = Twist()
	move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	motion_pub = rospy.Publisher("mobile_base/commands/velocity", Twist, queue_size=1)
	soundhandle.playWave(wavepath + "R2D2c.wav")
	rospy.sleep(1)
	seq_list = [None, None, 0, None]
	seq_cmd = ""
	print "I don't wanna get caught up in the rhythm of it"
	print "Say Somthing, say something !!!"
	ret = True
	speed = 0.2
	soundhandle.playWave(wavepath + "R2D2b.wav")
	while ret:
		time.sleep(0.1)
		print "The Command {0}".format(seq_cmd)
		response = recognize_speech_from_mic(recognizer, microphone) # {'transcription': u'Corner one', 'success': True, 'error': None}
		print "{0}".format(response)
		if response["transcription"]!=None and response["transcription"]!='':
			seq_cmd = match_responce(response, seq_cmd)
			if seq_cmd in VOCAB_DICT["GOAL_CMD"]:
				ret, seq_cmd = goal_motion(seq_cmd, move_base, cordinates, soundhandle, wavepath)
			elif seq_cmd in VOCAB_DICT["MOTION_CMD"]:
				ret, seq_cmd, move_msg = generic_motion(seq_cmd, move_msg, soundhandle, wavepath)
			else:
				pass
		else:
			soundhandle.playWave(wavepath + "R2D2b.wav")
			print "Couldn't hear you Try Again !"
		if seq_cmd !="":
			motion_pub.publish(move_msg)
		elif not ret:
			pass
		else:
			move_msg = Twist()
			motion_pub.publish(move_msg)
	print "Sometimes the greatest the way to say something is to say nothing at all \n\n\n!!!"
	soundhandle.playWave(wavepath + "R2D2c.wav")
	rospy.sleep(1)

	return 0

def using_my_code():
	cordinates = pd.read_csv("/home/{0}/catkin_ws/src/VishwakarmaS/res/assign3InitPos".format(getpass.getuser()))
	wavepath = "/home/{0}/catkin_ws/src/VishwakarmaS/res/".format(getpass.getuser())
	rospy.init_node("obeyer", anonymous=False)
	obeyer(cordinates, wavepath)
	return 0

def using_professors_package():
	model = '/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k'
	lexicon = '/home/{0}/catkin_ws/src/VishwakarmaS/res/voice_cmd.dic'.format(getpass.getuser())
	kwlist = '/home/{0}/catkin_ws/src/VishwakarmaS/res/voice_cmd.kwlist'.format(getpass.getuser())
	rospub = 'mobile_base/commands/velocity'
	rvc.ASRControl(model, lexicon, kwlist, rospub)
	return 0

def main():
	# using_my_code()
	using_professors_package()
	return 0

if __name__ == '__main__':
	main()
