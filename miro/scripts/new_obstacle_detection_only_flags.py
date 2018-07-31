#!/usr/bin/env python

import rospy

import std_msgs
from std_msgs.msg import Bool, Int32

import miro_msgs
from miro_msgs.msg import platform_sensors

from threading import Lock, Thread

class NewObstacleDetecor:

    def callback_safe(self, object):
        self.safe = object.data


    def callback_arrived(self, object):
        self.arrived = object.data

    def loop(self):
#	rate = rospy.Rate(1)
        while not rospy.is_shutdown():

		# when arrived is set to true by the oa node the robot must be safe 
		# otherwise the same obstacle will trigger a new obstacle detection
		if self.arrived: 
                   	self.obstacle_avoidance = False
		   	self.arrived = False
                   	print "obstacle_avoidance", self.obstacle_avoidance

		if not self.safe and not self.obstacle_avoidance:
			self.new_obstacle = True
			self.new_obstacle_pub.publish(self.new_obstacle)
			print "new obstacle", self.new_obstacle
                        self.new_obstacle = False
                        self.obstacle_avoidance = True
#	        rate.sleep()


    def __init__(self):   

        # attribute initialization
        self.safe = True
        self.arrived = True 
        self.obstacle_avoidance = False
        self.same_obstacle_counter = 0
        
        # subscribers
        rospy.Subscriber('/safe', Bool, self.callback_safe, queue_size = 1)
        rospy.Subscriber('/arrived', Bool, self.callback_arrived, queue_size = 1)
        
        # publishers
	self.new_obstacle_pub = rospy.Publisher('new_obstacle', Bool, queue_size = 0)
	
if __name__ == '__main__':
	rospy.init_node('new_obstacle_detection', anonymous = True)
        main = NewObstacleDetecor()
        main.loop()
