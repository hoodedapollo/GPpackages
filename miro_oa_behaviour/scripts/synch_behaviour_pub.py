#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String, Bool, Int64
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose,Vector3Stamped

import math
from math import pi
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## DESCRIPTION OF THE NODE
## This node produces a synchronous message by subscribing to two asynchronous messages
## When the asynchronous message /new_obstacle is recieved it means that the robot enters the 
## obstacle avoidance behaviour, while when the /arrived message is recieved it means that the
## robot has completed the oa behaviour. So when the robot is performing obstacle avoidance
## the message synch_oa_flag is published True synchronously. When the robot is not performing obstacle
## avoidance the message synch_oa_flag is published False synchronously.

class SynchOaBehaviour():

    def callback_new_obstacle( self, object):
        self.obstacle_avoidance = True

    def callback_arrived ( self, object):
        self.obstacle_avoidance= False

    def loop(self):
        
#        r = rospy.Rate(10)
        while not rospy.is_shutdown():

            if self.obstacle_avoidance:
                self.pub_synch_oa_flag.publish(True)

            else:
                self.pub_synch_oa_flag.publish(False)
#            r.sleep()

    def __init__(self):

        #attribute init
        self.obstacle_avoidance=False

        #subscribe
        self.sub_new_obstacle = rospy.Subscriber('/new_obstacle',Bool,self.callback_new_obstacle, queue_size=1)
        self.sub_arrived = rospy.Subscriber('/arrived',Bool,self.callback_arrived,queue_size=1)

        #publish
        self.pub_synch_oa_flag = rospy.Publisher ( '/synch_oa_flag', Bool, queue_size=0)

if __name__== '__main__':

    rospy.init_node('synch_oa_behaviour') 
    main = SynchOaBehaviour()
    main.loop()













