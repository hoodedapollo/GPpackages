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

class GestureBasedBehaviour():

	def __init__(self):

        #Last acelleration data received

		self.last_acc = [0,0,0]
		self.rpy = [0,0,0]

        #node rate
                self.rate = rospy.get_param('rate')

	#Parameter to set in the launch file to switch from the direct control ( linear acceleration ) to the attitude control ( roll pitch yaw )
		self.switch=rospy.get_param('~gb_control_type')
		
		self.sw_vel=Twist()
		self.body_vel=Twist()


		self.subAttitude = rospy.Subscriber('/rpy_deg',Vector3Stamped,self.callbackAttitude,queue_size=1)
		self.subLinear = rospy.Subscriber('/imu/data_raw',Imu,self.callbackLinear,queue_size=1)
		self.pub_gesture_control = rospy.Publisher('/gesture_based_behaviour', Twist, queue_size=0)
		

#Mapping from linear acceleration to MiRo body velocity	
	def callbackLinear(self,sm_data):

		self.last_acc[0] = sm_data.linear_acceleration.x
		self.last_acc[1] = sm_data.linear_acceleration.y
		self.last_acc[2] = sm_data.linear_acceleration.z
		
		
        
		if  -5 < self.last_acc[0] < 5 and -5 < self.last_acc[1] < 5:
			self.body_vel.linear.x=0.0
			self.body_vel.angular.z=0.0
			print ('MiRo STAY Still')
		
		elif -8 < self.last_acc[0] > 8:

			self.body_vel.linear.x=-self.last_acc[0]*50
			self.body_vel.angular.z=0.0
		
		else:

			self.body_vel.linear.x = -self.last_acc[0]*50
			self.body_vel.angular.z = -self.last_acc[1]*0.1
			print ('MiRo Go normal')
		
			
 
	def callbackAttitude(self,sw_data):

		self.rpy[0] = sw_data.vector.x
		self.rpy[1] = sw_data.vector.y
		self.rpy[2] = sw_data.vector.z


		newroll=self.rpy[0]+90

		if  -35 < newroll < 35 and -20 < self.rpy[1] < 20:
			self.sw_vel.linear.x=0.0
			self.sw_vel.angular.z=0.0
			print ('MiRo STAY Still')

		elif self.rpy[1] > 50 or self.rpy[1] < -60:
			self.sw_vel.linear.x=0.0#self.rpy[1]*6
			self.sw_vel.angular.z=0.0
			print ( 'max linear velocity')

		else:
			self.sw_vel.linear.x=self.rpy[1]*-7
			self.sw_vel.angular.z=-newroll*0.8*pi/180
			print ('MiRo follow your owner')


	#Switching between the two possible control based on the variable on the launch file
	def loop(self):
	        r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():

			if self.switch == 'linear':
				self.pub_gesture_control.publish(self.body_vel)
			if self.switch == 'attitude':
				self.pub_gesture_control.publish(self.sw_vel)
                        r.sleep()

    

if __name__=='__main__':
    rospy.init_node('attitude2body_vel')
    gesturecontrol = GestureBasedBehaviour()
    gesturecontrol.loop()
	
