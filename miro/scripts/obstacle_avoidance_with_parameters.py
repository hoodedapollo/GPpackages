#!/usr/bin/env python
#	@file
#	@section COPYRIGHT
#	Copyright (C) 2018 Consequential Robotics (CqR)
#	
#	@section AUTHOR
#	Consequential Robotics http://consequentialrobotics.com
#	
#	@section LICENSE
#	For a full copy of the license agreement, see LICENSE.MDK in
#	the MDK root directory.
#	
#	Subject to the terms of this Agreement, Consequential Robotics
#	grants to you a limited, non-exclusive, non-transferable license,
#	without right to sub-license, to use MIRO Developer Kit in
#	accordance with this Agreement and any other written agreement
#	with Consequential Robotics. Consequential Robotics does not
#	transfer the title of MIRO Developer Kit to you; the license
#	granted to you is not a sale. This agreement is a binding legal
#	agreement between Consequential Robotics and the purchasers or
#	users of MIRO Developer Kit.
#	
#	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
#	EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
#	OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
#	NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
#	HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
#	WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
#	FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
#	OTHER DEALINGS IN THE SOFTWARE.
#	
#	@section DESCRIPTION
#

################################################################

import rospy
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Twist, Quaternion

import nav_msgs
from nav_msgs.msg import Odometry

import tf

import miro_msgs
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream

import math
import numpy
import time
import sys
from miro_constants import miro


class miro_ros_client:

    def callback_platform_sensors(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_sensors = object

        # send downstream command, ignoring upstream data
        q = platform_control()

	if self.drive_pattern == "Pcontroller":
		d_ref = 0.5
		Kp =  1
		v = 100
		# wall on the right if d_sonar < d_ref --> e > 0 --> Kp > 0 since w > 0 turns left 
		# and moves away from the wall on the right
		error = (d_ref - self.platform_sensors.sonar_range.range)
		self.body_vel.angular.z = Kp * error
		# for now v is constant
		self.body_vel.linear.x = +v 
		print 'error', error
		print 'angular velocity (controller)' , self.body_vel.angular.z

	elif self.drive_pattern == "obstacle_avoidance":
		# the robot circumnavigates the new obstacle clockwise until it reaches the m-line
		if self.new_obstacle:

			#rotate head to the right
			self.body_config_speed[3] = -1 # maximum spped to yaw rotation
			self.body_config[3] = -1.5078 #yaw rotation 
			
			# publish
			q.body_vel = self.body_vel
			self.pub_platform_control.publish(q)

			#rotate 90 deg to the left
			while self.th < 1.5708 and not rospy.is_shutdown():
				self.body_vel.angualr.z = 1.5708/2 # rad/s
				
				# publish
				q.body_vel = self.body_vel
        			self.pub_platform_control.publish(q)
			

			# go around the obstacle simple proportionaol controller until m-line is met (y axis)
			# x_threshold is needed to avoid the robot thinks he is arrived when it is around the 
			# the initial position: y = 0, x = 0
			while self.x < x_threshold and self.y > y_threshold and not rospy.is_shutdown():

				# control parameters
				d_ref = 0.5
				Kp =  1
				v = 100
				
				# control action definition:
				# wall on the right if d_sonar < d_ref --> e > 0 --> Kp > 0 since w > 0 turns left 
				# and moves away from the wall on the right
				error = (d_ref - self.platform_sensors.sonar_range.range)
				self.body_vel.angular.z = Kp * error
				self.body_vel.linear.x = +v 
				print 'error', error
				print 'angular velocity (controller)', self.body_vel.angular.z
				
				# publish
				q.body_vel = self.body_vel
        			self.pub_platform_control.publish(q)

			#rotate head to look forward
			self.body_config_speed[3] = -1 # maximum spped to yaw rotation
			self.body_config[3] = 0 #yaw rotation 
			
			# publish
			q.body_vel = self.body_vel
			self.pub_platform_control.publish(q)
			
			#rotate to the left until the robot does not see the obstacle anymore
			# do not tell the robot to re-orient as zero theta since it may hit the obstacle with the tail
			while not safe and not rospy.is_shutdown():
				self.body_vel.angualr.z = 1.5708/2 # rad/s
				
				# publish
				q.body_vel = self.body_vel
        			self.pub_platform_control.publish(q)
				
			# once the obstacle avoidance is complete reset the new_obstacle flag waiting for 
			# the relative callback to update it 
			self.new_obstacle = False

        # publish
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

    def callback_new_obstacle(self, object):
	self.new_obstacle = object.data

    def callback_odometry(self, object):
	self.x = object.pose.pose.position.x
	self.y = object.pose.pose.position.y

	quaternion = object.pose.pose.orientation
	explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
	roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
        self.th = yaw       

    def loop(self):
        while True:
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tick"
            if rospy.core.is_shutdown():
                break
            time.sleep(1)
            print "tock"

    def __init__(self):
        
        # report
        print("initialising...")
        print(sys.version)

        # default data
        self.platform_sensors = None
        self.platform_state = None
        self.platform_mics = None
        self.core_state = None

        # pattern
        self.count = 0
        self.z_bak = -1
        self.body_vel = None
	self.body_config = None
	self.body_config_speed = None
	self.new_obstacle = None
	self.x = None
	self.y = None
	self.th = None

        # set inactive
        self.active = False

        # topic root
	self.robot_name = rospy.get_param('robot_name', 'sim01') # sim01 for simulation rob01 for real miro
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

	# drive pattern
	self.drive_pattern = rospy.get_param('~drive_pattern', 'obstacle_avoidance') #PIDcontroller for online pid tuning of angular velocity gains
        print "drive_pattern", self.drive_pattern
        
        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)

        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)

	self.sub_new_obstacle = rospy.Subscriber("/new_obstacle", Bool, self.callback_new_obstacle)

	self.sub_odometry = rospy.Subscriber("/odom", Odometry, self.callback_odometry)

        # set active
        self.active = True

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_with_parameters", anonymous=True)
    main = miro_ros_client()
    main.loop()
