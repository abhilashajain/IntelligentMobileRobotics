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


VOCAB_DICT = {"MOTION_CMD": ["stop","back","forward","backward","left","right","rotate"],\
				"GOAL_CMD": ["corner one","corner two","corner three","corner four","outside door","quit"],\
				"DECISION_CMD":["yes", "no"]}


def recognize_speech_from_mic(recognizer, microphone):
	"""Transcribe speech from recorded from `microphone`.

	Returns a dictionary with three keys:
	"success": a boolean indicating whether or not the API request was
			   successful
	"error":   `None` if no error occured, otherwise a string containing
			   an error message if the API could not be reached or
			   speech was unrecognizable
	"transcription": `None` if speech could not be transcribed,
			   otherwise a string containing the transcribed text
	"""
	# # check that recognizer and microphone arguments are appropriate type

	if not isinstance(recognizer, sr.Recognizer):
		raise TypeError("`recognizer` must be `Recognizer` instance")

	if not isinstance(microphone, sr.Microphone):
		raise TypeError("`microphone` must be `Microphone` instance")

	# # adjust the recognizer sensitivity to ambient noise and record audio
	# from the microphone
	start_time = time.time()
	with microphone as source:
		recognizer.adjust_for_ambient_noise(source, duration=1)
		# recognizer.record(source)
		audio = recognizer.listen(source)
	print "Audio Capturing --- %s seconds ---" % (time.time() - start_time)
	# set up the response object
	response = {
		"success": True,
		"error": None,
		"transcription": None
		# "transcription": u'Corner one',
	}

	# try recognizing the speech in the recording
	# if a RequestError or UnknownValueError exception is caught,
	#     update the response object accordingly
	try:
		start_time = time.time()
		response["transcription"] = recognizer.recognize_google(audio)
		# response["transcription"] = recognizer.recognize_sphinx(audio)
		print "Audio Recognize --- %s seconds ---" % (time.time() - start_time)
	except sr.RequestError:
		# API was unreachable or unresponsive
		response["success"] = False
		response["error"] = "API unavailable"
	except sr.UnknownValueError:
		# speech was unintelligible
		response["success"] = False
		response["error"] = "Unable to recognize speech"

	return response


def generic_motion(seq_cmd, move_msg, soundhandle, wavepath):

	# if seq_list[-1]=="yes":
	if seq_list[1]=="forward":
		move_msg.linear.x = 0.4
		move_msg.angular.z = 0.0
	elif seq_list[1]=="backward":
		move_msg.linear.x = -0.4
		move_msg.angular.z = 0.0
	elif seq_list[1]=="stop":
		move_msg.linear.x = 0.0
		move_msg.angular.z = 0.0
	elif seq_list[1]=="back":
		move_msg.linear.x = -0.5
		move_msg.angular.z = 0.0
	elif seq_list[1]=="left":
		move_msg.angular.z = 0.10
	elif seq_list[1]=="right":
		move_msg.angular.z = -0.10
	elif seq_list[1]=="rotate":
		move_msg.linear.x = 0.0
		move_msg.angular.z = 1
	else:
		# pass
	# elif seq_list[-1]=="no":
		# seq_list = [None, None, 0, None] # cancel all the commands
	# else:
		soundhandle.playWave(wavepath + "R2D2b.wav")
		# print "Do you want me to go {0}".format(seq_list[1])
		# print "Say Yes or No !!!"
	return True, seq_cmd, move_msg


def goal_motion(seq_cmd, move_base, cordinates, soundhandle, wavepath):

	# if seq_list[-1]=="yes":
	if seq_cmd !="quit":
		x, y, z, w = td.get_goal_cordinates(seq_list[1], cordinates, True)
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
	# elif seq_list[-1]=="no":
	# 	seq_list = [None, None, 0, None] # cancel all the commands
	# else:
	# 	print "Do you want me to go {0}".format(seq_list[1])
	# 	print "Say Yes or No !!!"
	return True, seq_cmd


def match_responce(response, seq_cmd):
	cmd_found = False
	# if seq_list[0]==None or seq_list[2]==0:
	for cmd in VOCAB_DICT["MOTION_CMD"]:
		ratio = SequenceMatcher(None, cmd, response["transcription"]).ratio()
		if ratio > 0.7:
			cmd_found = True
			# seq_list[0] = "generic_motion"
			# seq_list[1] = cmd.lower()
			seq_cmd = cmd.lower()
			# seq_list[2] = 1
	if not cmd_found:
		for cmd in VOCAB_DICT["GOAL_CMD"]:
			ratio = SequenceMatcher(None, cmd, response["transcription"]).ratio() # "Go to Corner One", "forner One" ratio 0.699
			if ratio > 0.7:
				cmd_found = True
				seq_cmd = cmd.lower()
				# seq_list[0] = "goal_motion"
				# seq_list[1] = cmd.lower()
				# seq_list[2] = 1
		if not cmd_found:
			print "Can You be more clear next time !"
	# else:
	# 	if SequenceMatcher(None, "yes", response["transcription"]).ratio() > 0.7:
	# 		seq_list[-1] =  "yes"
	# 		seq_list[2] = 0
	#
	# 	elif SequenceMatcher(None, "no", response["transcription"]).ratio() > 0.7:
	# 		seq_list[-1] =  "no"
	# 		seq_list[2] = 0
	# 	else:
	# 		print "Do you want me to go {0}".format(seq_list[0])
	# 		print "Say Yes or No !!!"

	return seq_cmd


def obeyer(cordinates, wavepath):
	soundhandle = SoundClient()
	# soundhandle = None
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
		# seq_cmd = u'forward'
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
		# time.sleep(0.5)
	print "Sometimes the greatest the way to say something is to say nothing at all \n\n\n!!!"
	soundhandle.playWave(wavepath + "R2D2c.wav")
	rospy.sleep(1)

	return 0


def main():
	cordinates = pd.read_csv("/home/{0}/catkin_ws/src/VishwakarmaS/res/assign3InitPos".format(getpass.getuser()))
	wavepath = "/home/{0}/catkin_ws/src/VishwakarmaS/res/".format(getpass.getuser())
	rospy.init_node("obeyer", anonymous=False)
	obeyer(cordinates, wavepath)
	return 0


if __name__ == '__main__':
	main()
