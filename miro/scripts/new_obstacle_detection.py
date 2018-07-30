#!/usr/bin/env python

import rospy

import std_msgs
from std_msgs.msg import Bool, Int32

import miro_msgs
from miro_msgs.msg import platform_sensors

class NewObstacleDetecor:

    def callback_safe(self, object):
        self.safe = object.data

    def callback_arrived(self, object):
        self.arrived = object.data
        
    def loop():
        while not rospy_is_shutdown():

            if arrived: 
                self.same_obsatcle_counter = 0

            if not safeSub.safe:
                self.same_obstacle_counter+= 1

            # publish new_obstacle only once with the value True each time a new obstacle is encountered
            # otherwise no messages are published (no callback are called by the subscribers)
            if sameObstacle.counter == 1: 
                new_obstacle = True
                newObstPub.pub.publish(new_obstacle)

    def __init__():   

        # attribute initialization
        self.safe = True
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
