#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose,Vector3Stamped

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import tf

import math
from math import pi
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

##DESCRIPTION OF THE NODE

## Subscribe to the topic /switching_behaviour
## read from that topic the values of safety_msg
## Subscribe to the topic /sw_mapping 
## if safety_msg != 0 publish on linear.x and angular.z the value published on /sw_mapping

class Conversionrad2deg():

    def __init__(self):

        self.rpy = Vector3Stamped()
        
        self.subSonar = rospy.Subscriber('/imu/rpy/filtered',Vector3Stamped,self.callbackConversion,queue_size=1)
        self.pub_deg = rospy.Publisher('/rpy_deg',Vector3Stamped,queue_size=0)

            

    def callbackConversion(self,rpy_v): 

        self.rpy.vector.x=rpy_v.vector.x
        self.rpy.vector.y=rpy_v.vector.y
        self.rpy.vector.z=rpy_v.vector.z

        rpy_deg=Vector3Stamped()
        rpy_deg.vector.x=self.rpy.vector.x*180/pi
        rpy_deg.vector.y=self.rpy.vector.y*180/pi
        rpy_deg.vector.z=self.rpy.vector.z*180/pi
        self.pub_deg.publish(rpy_deg)


    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('rad2deg')
    rad2deg = Conversionrad2deg()
    rad2deg.main()