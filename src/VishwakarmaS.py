#!/usr/bin/env python
"""
Script to draw Initials with Turtlesim
"""
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
import os


def log_pos(pos_data):
	with open('log_file.log', 'a+') as opened_file:
		opened_file.write("X Cordinate at {0}\n".format(pos_data.x))
		opened_file.write("Y Cordinate atat {0}\n".format(pos_data.y))
		opened_file.write("Theta at {0}\n".format(pos_data.theta))
		opened_file.write("Linear_velocity at {0}\n".format(pos_data.linear_velocity))
		opened_file.write("Angular_velocity at {0}\n".format(pos_data.angular_velocity))
		opened_file.write("//////////////////////\n")
	print "X Cordinate at {0}\n".format(pos_data.x)
	print "Y Cordinate atat {0}\n".format(pos_data.y)
	print "Theta at {0}\n".format(pos_data.theta)
	print "Linear_velocity at {0}\n".format(pos_data.linear_velocity)
	print "Angular_velocity at {0}\n".format(pos_data.angular_velocity)
	# rospy.loginfo("I heard %s", pos_data.data)
	# rospy.loginfo(rospy.get_caller_id() + "I heard %s", pos_data)
	return 0

def publish_move(pub, sub, move_msg):
	pub.publish(move_msg)
	return 0

def publish_ntimes(num, pub, sub, move_msg):
	for _ in range(num):
		publish_move(pub, sub, move_msg)
		time.sleep(1.2)
	return 0

def create_s(pub, sub, move_msg):
	move_msg.linear.x = -1.0
	move_msg.angular.z = -1.0
	publish_ntimes(2, pub, sub, move_msg)
	move_msg.linear.x = 1.0
	move_msg.angular.z = 1.0
	publish_ntimes(4, pub, sub, move_msg)
	move_msg.angular.z = -1.0
	publish_ntimes(4, pub, sub, move_msg)
	move_msg.angular.z = -0.2
	move_msg.linear.x = 0.2
	publish_ntimes(1, pub, sub, move_msg)

	return 0

def create_v(pub, sub, move_msg):
	move_msg.angular.z = 0.0
	move_msg.linear.x = 1.0
	publish_ntimes(3, pub, sub, move_msg)
	move_msg.linear.x = 0.5
	publish_ntimes(1, pub, sub, move_msg)
	move_msg.angular.z = 2.50
	publish_ntimes(1, pub, sub, move_msg)
	move_msg.angular.z = 0.0
	publish_ntimes(1, pub, sub, move_msg)
	move_msg.linear.x = 1.0
	publish_ntimes(3, pub, sub, move_msg)

	return 0

def main():
	with open('log_file.log', 'w') as opened_file:
		opened_file.write("THIS IS VishwakarmaS Package Ros Log: Assign 1\n\n")
		opened_file.write("Working Dir : {0}\n".format(os.getcwd()))
	rospy.init_node('sagar_turtlesim')
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
	# pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=1) # to move actual turtlebot use this node to publish move msg
	sub = rospy.Subscriber('/turtle1/pose', Pose, log_pos)
	move_msg = Twist()
	create_s(pub, sub, move_msg)
	create_v(pub, sub, move_msg)
	return 0

if __name__ == '__main__':
	main()
