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

    def new_obst_init(self):	
		if self.arrived: 
		     self.lock.acquire()
		     self.same_obsatcle_counter = 0
	             self.lock.release()
		     self.arrived = False
		     print "same_obstacle_counter", self.same_obsatcle_counter

    def same_obst_counter(self):
		if not self.safe:
		    self.lock.acquire()
		    self.same_obstacle_counter += 1
	            self.lock.release()
		    print "same_obstacle_counter", self.same_obstacle_counter




    def loop(self):
	threads = []
	for func in [self.new_obst_init, self.same_obst_counter]:
		threads.append(Thread(target=func))
		threads[-1].start()
	rate = rospy.Rate(1)
        while not rospy.is_shutdown():
		    for thread in threads:
			thread.join() 

		    # publish new_obstacle only once with the value True each time a new obstacle is encountered
		    # otherwise no messages are published (no callback are called by the subscribers)
		    if self.same_obstacle_counter == 1: 
			self.new_obstacle = True
			self.new_obstacle_pub.publish(self.new_obstacle)
			print "new obstacle", self.new_obstacle
		    rate.sleep()


    def __init__(self):   

        # attribute initialization
        self.safe = True
        self.arrived = True 
        self.same_obstacle_counter = 0
        
        # subscribers
        rospy.Subscriber('/safe', Bool, self.callback_safe, queue_size = 1)
        rospy.Subscriber('/arrived', Bool, self.callback_arrived, queue_size = 1)
        
        # publishers
	self.new_obstacle_pub = rospy.Publisher('new_obstacle', Bool, queue_size = 0)
	
	#mutex
	self.lock = Lock()


if __name__ == '__main__':
	rospy.init_node('new_obstacle_detection', anonymous = True)
        main = NewObstacleDetecor()
        main.loop()
