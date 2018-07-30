#!/usr/bin/env python

import rospy

import std_msgs
from std_msgs.msg import Float64

import miro_msgs
from miro_msgs.msg import platform_sensors

import time

class SonarSetPoint:

	def loop(self):
                rate = rospy.Rate(50) # 50hz
		while not rospy.core.is_shutdown():
                    self.pub_sonar_distance_setpoint.publish(self.sonar_distance_setpoint)
                    rate.sleep()
	

        def __init__(self):
		# attributes initialization
		self.sonar_distance_setpoint = rospy.get_param('sonar_distance_setpoint', 0.5) 

		#publish
		self.pub_sonar_distance_setpoint = rospy.Publisher('/setpoint', Float64, queue_size = 0)


if __name__ == '__main__':
	rospy.init_node('sonar_setpoint', anonymous = True)
	main = SonarSetPoint()
	main.loop()
