#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose


import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream


import math
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

## Subscribe to the topic /inertial
## read from that topic the values of linear acceleration
## Publish to the topic /miro/rob01/platform_control
## write on linear.x and angular.z 

class Smartwatch_data():
    def __init__(self):

        #Last acelleration data received
        self.last_acc = [0,0,0]
        self.subSmartwatch = rospy.Subscriber('/inertial',Imu,self.callbackSmartwatch,queue_size=1)
        self.pub_platform_control = rospy.Publisher('/miro/rob01/platform/control', platform_control, queue_size=0)

    def callbackSmartwatch(self,sm_data):

        self.last_acc[0] = sm_data.linear_acceleration.x
        self.last_acc[1] = sm_data.linear_acceleration.y
        self.last_acc[2] = sm_data.linear_acceleration.z

        #The map between the acceleration value from the sw to the velocity value on MiRo must follow the following rules:
        # 1) when linear acceleration x and linear acceleration z are under a threshold () we want that linear and angular acceleration on MiRo=0
        # 2.1) when linear acceleration x is max ( >10 ) we want the linear velocity of MiRo=full speed ( 250 mm/s)
        # 2.2) when linear acceleration z is max ( >10 ) we want the angular velocity of MiRo = pi/4

        q=platform_control()
        self.body_vel=Twist()

        #1)
        if  -2 < self.last_acc[0] < 2 and -2 < self.last_acc[1] < 2:
                self.body_vel.linear.x=0.0
                self.body_vel.angular.z=0.0
                print ('MiRo STAY Still')
                q.lights_raw = [0,255,255,0,255,255,0,255,255,0,255,255,0,255,255,0,255,255]

        if self.last_acc[1] > 0 and self.last_acc[1]>2:
            q.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,0,0]

        if self.last_acc[1] < 0 and self.last_acc[1]<-2:
            q.lights_raw = [0,0,0,0,0,0,0,0,0,255,0,255,0,0,255,0,0,255]

        #elif self.last_acc[0]>5.0:
            #q.lights_raw = [0,255,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0]

        # #2.1)
        # elif self.last_acc[0]> 7.5 and self.last_acc[1]> 8:
        #     self.body_vel.linear.x=+200
        #     self.body_vel.angular.z=+0.785398
        #     print ('MiRo Max speed')

        # #2.2)
        # elif self.last_acc[0]< -7.5 and self.last_acc[1]< -8:
        #     self.body_vel.linear.x=-200
        #     self.body_vel.angular.z=-0.785398
        #     print ('MiRo Max neg speed')
        # else: 

        self.body_vel.linear.x=self.last_acc[0]*50
        self.body_vel.angular.z=self.last_acc[1]*0.5
        print ('MiRo Go normal')
    
        #q.eyelid_closure = 1
        #q.eyelid_closure = 0
        #q.lights_raw = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

    def main (self):
        rospy.spin()

if __name__=='__main__':
    rospy.init_node('miro_smartwatch')
    smartwatch = Smartwatch_data()
    smartwatch.main()

