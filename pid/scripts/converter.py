#!/usr/bin/env python

import rospy

import std_msgs
from std_msgs.msg import Float64

import miro_msgs
from miro_msgs.msg import platform_sensors

import time

class Converter:

	#subscribers callbacks
	def sonar_range_callback(self, object):
		self.sonar_distance = object.sonar_range.range
		
		self.pub_sonar_distance.publish(self.sonar_distance)

	def loop(self):
		while True:
		    if rospy.core.is_shutdown():
			break
		    time.sleep(1)
		    print "tick"
		    if rospy.core.is_shutdown():
			break
		    time.sleep(1)
		    print "tock"

	

        def __init__(self):
		# attributes initialization
		self.sonarDistance = 0

		#publish
		self.pub_sonar_distance = rospy.Publisher('/state', Float64, queue_size = 0)

		# subscribers initialization 
		self.sub_platform_sensors = rospy.Subscriber('miro/sim01/platform/sensors', platform_sensors, self.sonar_range_callback, queue_size = 1)

if __name__ == '__main__':
	rospy.init_node('converter', anonymous = True)
	main = Converter()
	main.loop()
