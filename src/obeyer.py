#!/usr/bin/env python
"""
Script to move to Map location with voice commands - Assignment 3 Part 2
"""
import rospy
import getpass
import actionlib
import pandas as pd
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
import speech_recognition as sr
from geometry_msgs.msg import Point
from sound_play.libsoundplay import SoundClient
import time



VOCAB_DICT = {"MOTION_CMD": ["Stop","Back","Forward","Backward","Left","Right","Rotate"],\
				"GOAL_CMD": ["Corner One","Corner Two","Corner Three","Corner Four","Outside Door","Quit"]}

#
# class Obeyer:
# 	def __init__(self, cordinates):
#
#
# 		# self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
# 		# rospy.on_shutdown(self.cleanup)
# 		# self.voice = rospy.get_param("~voice", "voice_don_diphone")
# 		self.wavepath = "/home/{0}/catkin_ws/src/VishwakarmaS/res/".format(getpass.getuser())
# 		# Create the sound client object
# 		self.soundhandle = SoundClient()
# 		rospy.sleep(1)
# 		self.soundhandle.stopAll()
# 		# Announce that we are ready for input
# 		self.soundhandle.playWave(self.wavepath + "R2D2b.wav")
# 		rospy.sleep(1)
# 		# self.soundhandle.say("Ready", self.voice)
# 		rospy.loginfo("Say one of the navigation commands...")
# 		# Subscribe to the recognizer output
# 		rospy.Subscriber('/recognizer/output', String, self.confirm_cmd)
#
# 	def confirm_cmd(self, msg):
# 		rospy.loginfo('You Said :: {0}'.format(msg.data))
# 		self.soundhandle.playWave(self.wavepath + "R2D2c.wav")
# 		rospy.sleep(1)
#
# 		return 0
#
# 	def move_base(self):
# 		pass
#
# 	def get_goal_location(self, goal_string):
# 		pass

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
		# recognizer.adjust_for_ambient_noise(source, duration=0.5)
		recognizer.record(source, duration=2)
		audio = recognizer.listen(source)

	print "Audio Capturing --- %s seconds ---" % (time.time() - start_time)
	# set up the response object
	response = {
		"success": True,
		"error": None,
		"transcription": None
	}

	# try recognizing the speech in the recording
	# if a RequestError or UnknownValueError exception is caught,
	#     update the response object accordingly
	try:
		start_time = time.time()
		# response["transcription"] = recognizer.recognize_google(audio)
		response["transcription"] = recognizer.recognize_sphinx(audio)
		print "Audio Recognize --- %s seconds ---" % (time.time() - start_time)
	except sr.RequestError:
		# API was unreachable or unresponsive
		response["success"] = False
		response["error"] = "API unavailable"
	except sr.UnknownValueError:
		# speech was unintelligible
		response["error"] = "Unable to recognize speech"

	return response


def obeyer(cordinates, wavepath):
	soundhandle = SoundClient()
	recognizer = sr.Recognizer()
	microphone = sr.Microphone()
	while True:
		print "I don't wanna get caught up in the rhythm of it"
		print "Say Somthing, say something !!!"
		time.sleep(0.2)
		response = recognize_speech_from_mic(recognizer, microphone)
		# NOTE:
		# check is recorded is sucess
		# check if word falls under dictionary above
		# if yes play a sound a record yes or no to go ahed with the command else discard the previous commands
		# in this case mentain what was said last or what goal was set

		print "{0}".format(response)

		print "Sometimes the greatest the way to say something is to say nothing at all \n\n\n!!!"
		time.sleep(0.5)

	return 0


def main():
	cordinates = pd.read_csv("/home/{0}/catkin_ws/src/VishwakarmaS/res/assign3InitPos".format(getpass.getuser()))
	wavepath = "/home/{0}/catkin_ws/src/VishwakarmaS/res/".format(getpass.getuser())
	rospy.init_node("obeyer", anonymous=False)
	obeyer(cordinates, wavepath)
	return 0


if __name__ == '__main__':
	main()
