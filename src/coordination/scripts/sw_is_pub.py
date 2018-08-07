#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String,Bool,Float32
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose

import math
import numpy
import time
import sys
from miro_constants import miro
import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state


from datetime import datetime

class IsPublishing():

        def __init__(self):

            self.sw_state = False
            self.dt = 1.0 #Must be more than period between two Imu data msgs
            self.initial_time = rospy.Time.now().to_sec()
            self.t_last_cb = rospy.Time.now().to_sec()+self.dt
            self.rate = rospy.get_param("rate",10)
            self.subSwState = rospy.Subscriber ( '/imu/data_raw',Imu,self.callbackSwState,queue_size=1)
            self.pub_sw_state = rospy.Publisher ( '/is_publishing',Bool,queue_size=0)

        def callbackSwState ( self, object ):

            self.t_last_cb = object.header.stamp.to_sec() 
        
        def loop(self):
            r = rospy.Rate(self.rate)
            while not rospy.is_shutdown():
                
                if rospy.Time.now().to_sec() > self.t_last_cb + self.dt:

                    self.sw_state = False

                else:
                
                    self.sw_state = True
                rospy.loginfo('rospy.Time.now: %s t_last_cb+dt: %s sw_state: %s', rospy.Time.now().to_sec(),self.t_last_cb+self.dt,self.sw_state)
                self.pub_sw_state.publish(self.sw_state)
                r.sleep()


if __name__== '__main__':

        rospy.init_node('sw_is_pub')
        ip = IsPublishing()
        ip.loop()

