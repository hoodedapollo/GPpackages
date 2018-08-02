#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

from mapping_msgs.msg import mapping, safety
import math
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

class GestureBased():

    def __init__(self):
        self.safety_flag=0

        self.subSonar = rospy.Subscriber('/switching_behaviour',safety,self.callbackSafe,queue_size=1)
        self.subSonar = rospy.Subscriber('/sw_mapping',mapping,self.callbackGBB,queue_size=1)
        self.pub_platform_control = rospy.Publisher('/miro/rob01/platform/control', platform_control, queue_size=0)
        #if self.safety_flag !=0:
            

    def callbackSafe(self,safety_msg): 

        self.safety_flag=safety_msg.safe

    def callbackGBB(self,vel_msg):

        if self.safety_flag!=0:

            print "GBB MODE ON"

            #self.vel_msg=Twist()   

            q=platform_control()
            self.body_vel=Twist()
            self.body_vel.linear.x = vel_msg.sw_vel.linear.x
            self.body_vel.angular.z = vel_msg.sw_vel.angular.z
        
        
            q.body_vel = self.body_vel

            #lightening pattern

            if -5< vel_msg.sw_vel.linear.x < 5 and -0.1 < vel_msg.sw_vel.angular.z<0.1 :
                q.lights_raw = [0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255,255]

            if vel_msg.sw_vel.angular.z > 0 and vel_msg.sw_vel.angular.z > 0.1 :
                q.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0]


            if vel_msg.sw_vel.angular.z < 0 and vel_msg.sw_vel.angular.z < - 0.1 :
                q.lights_raw = [0,0,0,0,0,0,0,0,0,255,0,255,0,0,255,0,0,255]



            self.pub_platform_control.publish(q)
            

        
        #rospy.loginfo(sonar_msg.sonar_range)

    def main (self):
        rospy.spin()

if __name__== '__main__':
    rospy.init_node('gbb_miro')
    gesture_base = GestureBased()
    gesture_base.main()