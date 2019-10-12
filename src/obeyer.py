#!/usr/bin/env python
"""
Script to move to Map location with voice commands - Assignment 3
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
