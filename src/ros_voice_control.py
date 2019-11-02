#!/usr/bin/env python

"""This module is a simple demonstration of voice control
for ROS turtlebot using pocketsphinx
"""

import argparse
import roslib
import rospy
import getpass

from geometry_msgs.msg import Twist

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio


class ASRControl(object):
	"""Simple voice control interface for ROS turtlebot

	Attributes:
		model: model path
		lexicon: pronunciation dictionary
		kwlist: keyword list file
		pub: where to send commands (default: 'mobile_base/commands/velocity')

	"""
	def __init__(self, model, lexicon, kwlist, pub):
		# initialize ROS
		self.speed = 0.2
		self.msg = Twist()

		rospy.init_node('obeyer', anonymous=False)
		rospy.on_shutdown(self.shutdown)

		# you may need to change publisher destination depending on what you run
		self.pub_ = rospy.Publisher(pub, Twist, queue_size=10)

		# initialize pocketsphinx
		config = Decoder.default_config()
		config.set_string('-hmm', model)
		config.set_string('-dict', lexicon)
		config.set_string('-kws', kwlist)

		stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
						rate=16000, input=True, frames_per_buffer=1024)
		stream.start_stream()

		self.decoder = Decoder(config)
		self.decoder.start_utt()

		while not rospy.is_shutdown():
			buf = stream.read(1024)
			if buf:
				self.decoder.process_raw(buf, False, False)
			else:
				break
			self.parse_asr_result()

	def parse_asr_result(self):
		"""
		move the robot based on ASR hypothesis
		"""
		if self.decoder.hyp() != None:
			print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
				for seg in self.decoder.seg()])
			print ("Detected keyphrase, restarting search")
			seg.word = seg.word.lower()
			self.decoder.end_utt()
			self.decoder.start_utt()
			# you may want to modify the main logic here
			if seg.word.find("forward") > -1:
				self.msg.linear.x = self.speed
				self.msg.angular.z = 0
			elif seg.word.find("left") > -1:
				if self.msg.linear.x != 0:
					if self.msg.angular.z < self.speed:
						self.msg.angular.z += 0.05
				else:
					self.msg.angular.z = self.speed*2
			elif seg.word.find("right") > -1:
				if self.msg.linear.x != 0:
					if self.msg.angular.z > -self.speed:
						self.msg.angular.z -= 0.05
				else:
					self.msg.angular.z = -self.speed*2
			elif seg.word.find("back") > -1:
				self.msg.linear.x = -self.speed
				self.msg.angular.z = 0
			elif seg.word.find("stop") > -1 or seg.word.find("halt") > -1:
				self.msg = Twist()
		print "self.msg :: {0}".format(self.msg)
		self.pub_.publish(self.msg)

	def shutdown(self):
		"""
		command executed after Ctrl+C is pressed
		"""
		rospy.loginfo("Stop ASRControl")
		self.pub_.publish(Twist())
		rospy.sleep(1)


def main():

	model = '/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k'
	lexicon = '/home/{0}/catkin_ws/src/VishwakarmaS/res/voice_cmd.dic'.format(getpass.getuser())
	kwlist = '/home/{0}/catkin_ws/src/VishwakarmaS/res/voice_cmd.kwlist'.format(getpass.getuser())
	rospub = 'mobile_base/commands/velocity'
	ASRControl(model, lexicon, kwlist, rospub)

	return 0


if __name__ == '__main__':
	main()
