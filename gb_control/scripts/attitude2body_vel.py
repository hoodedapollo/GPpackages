#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose,Vector3Stamped

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
from math import pi
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

##DESCRIPTION OF THE NODE

## Subscribe to the topic /rad2deg that gives roll pitch and yaw in degress
## read from that topic the values of attitude ( roll - pitch - yaw)
## map those data and publish a message on miro that contains those mapped values
## read the value along z and publish a variable human influence 

class rpy2body_vel():

    def __init__(self):

        #Last acelleration data received
        self.rpy = [0,0,0]

        self.subSmartwatch = rospy.Subscriber('/rpy_deg',Vector3Stamped,self.callbackrpy,queue_size=1)
        self.pub_mapping = rospy.Publisher('/miro/sim01/platform/control', platform_control, queue_size=0)

    def callbackimu(self,imux):

        self.gyro_vel=imux.angular_velocity.z
 

    def callbackrpy(self,sw_data):

        self.rpy[0] = sw_data.vector.x#to modify
        self.rpy[1] = sw_data.vector.y#to modify
        self.rpy[2] = sw_data.vector.z#to modify


        q=platform_control()
        self.sw_vel=Twist()

        newroll=self.rpy[0]+70

        #1) TO DO: MODIFY THE THRESHOLD
        if  -15 < newroll < 15 and -20 < self.rpy[1] < 20:
            self.sw_vel.linear.x=0.0
            self.sw_vel.angular.z=0.0
            print ('MiRo STAY Still')

        elif self.rpy[1] > 40 or self.rpy[1] < -60:

            self.sw_vel.linear.x=self.rpy[1]*6
            self.sw_vel.angular.z=0.0
            print ( 'max linear velocity')


        #if self.rpy[2] > 60:

        #self.lastyaw=True
 
        else:

            self.sw_vel.linear.x=0.0
            self.sw_vel.angular.z=-newroll*0.5*pi/180
            print ('MiRo follow your owner')


        q.body_vel = self.sw_vel
        self.pub_mapping.publish(q)

    def main (self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('attitude2body_vel')
    attitudebasecontrol = rpy2body_vel()
    attitudebasecontrol.main()
