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
from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Twist

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

        # timing
        sync_rate = 50 # miro by default publishes at 50 Hz
        period = 2 * sync_rate # two seconds per period
        z = self.count / period # when z == 1 means that self.count is 100 which at 50 Hz means 2 seconds which is the period
       				# In Python 2.x, integer divisions truncates instead of becoming a floating point number: 1 / 2 = 0 
        # advance pattern
        if not z == self.z_bak:
            self.z_bak = z
            
            # create body_vel for next pattern segment
            self.body_vel = Twist()
            if self.drive_pattern == "square":
                # square dance
                if z == 0 or z == 2:
                    print "turn left"
                    self.body_vel.angular.z = +0.7854
                if z == 1 or z == 3:
                    print "drive forward"
                    self.body_vel.linear.x = +200

	        elif self.drive_pattern == "safety":
                    if 0.03 < self.platform_sensors.sonar_range < 0.7:
                        self.body_vel.linear.x = -200
                    else:
                        self.body_vel.linear.x = +200

	    elif self.drive_pattern == "safety":
		if 0.03 < self.platform_sensors.sonar_range.range < 0.7:
			self.body_vel.linear.x = -200
		else:
			self.body_vel.linear.x = 50
		print self.platform_sensors.sonar_range.range
	    
	    elif self.drive_pattern == "avoidance1":
		# if there is something withing the safety threshold rotate left (w > 0)
		if 0.03 < self.platform_sensors.sonar_range.range < 0.7:
			self.body_vel.angular.z = +1.5708
		# else go straight on
		else:
			self.body_vel.linear.x = +200 
	    
	    elif self.drive_pattern == "avoidance2":
		w = 1.5708/2
		v = 200
		# if measured distance is too close to the wall 
		if 0.03 < self.platform_sensors.sonar_range.range < 0.7:
			#take a small right and move staright
			self.body_vel.angular.z = -w
		else: 
			#take a small left and move staright
			self.body_vel.angular.z = +w

		# publish the roation vommand
		q.body_vel = self.body_vel
		self.pub_platform_control.publish(q)

		self.body_vel.linear.x = +v 
		
	    elif self.drive_pattern == "Pcontroller":
		d_ref = 0.5
		Kp =  0.25 / 0.5
		v = 200
		# wall on the right if d_sonar < d_ref --> e > 0 --> Kp > 0 since w > 0 turns left and moves away from the wall on the right
		error = (d_ref - self.platform_sensors.sonar_range.range)
		self.body_vel.angular.z = Kp * error
		# for now v is constant
		self.body_vel.linear.x = +v 
		print error
		print self.body_vel.angular.z

	    elif self.drive_pattern == "PIDcontroller":
		v = 100
		self.body_vel.angular.z = self.control_effort

            else:
                # do-si-do
                if z == 0:
                    print "turn left"
                    self.body_vel.angular.z = +1.5708
                if z == 1:
                    print "drive forward"
                    self.body_vel.linear.x = +200
                if z == 2:
                    print "turn right"
                    self.body_vel.angular.z = -1.5708
                if z == 3:
                    print "drive forward"
                    self.body_vel.linear.x = +200
        
		# point cameras down
		#q.body_config[1] = 1.0
		#q.body_config_speed[1] = miro.MIRO_P2U_W_LEAN_SPEED_INF

        # publish
        q.body_vel = self.body_vel
        self.pub_platform_control.publish(q)

        # count
        self.count = self.count + 1
        if self.count == 400:
            self.count = 0
        

    def callback_platform_state(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_state = object

    def callback_platform_mics(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.platform_mics = object

    def callback_core_state(self, object):
        
        # ignore until active
        if not self.active:
            return

        # store object
        self.core_state = object

    def callback_pid_control(self,object):
	self.control_effort = object.data

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
	
	self.control_effort = 0

        # pattern
        self.count = 0
        self.z_bak = -1
        self.body_vel = None

        # set inactive
        self.active = False

        # topic root
	self.robot_name = rospy.get_param('robot_name', 'sim01')
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

	# drive pattern
	self.drive_pattern = rospy.get_param('drive_pattern', 'square')
        print "drive_pattern", self.drive_pattern
        
        # publish
        self.pub_platform_control = rospy.Publisher(topic_root + "/platform/control",
                    platform_control, queue_size=0)
        self.pub_core_control = rospy.Publisher(topic_root + "/core/control",
                    core_control, queue_size=0)
        self.pub_core_config = rospy.Publisher(topic_root + "/core/config",
                    core_config, queue_size=0)
        self.pub_bridge_config = rospy.Publisher(topic_root + "/bridge/config",
                    bridge_config, queue_size=0)
        self.pub_bridge_stream = rospy.Publisher(topic_root + "/bridge/stream",
                    bridge_stream, queue_size=0)
        self.pub_platform_config = rospy.Publisher(topic_root + "/platform/config",
                    platform_config, queue_size=0)

        # subscribe
        self.sub_sensors = rospy.Subscriber(topic_root + "/platform/sensors",
                platform_sensors, self.callback_platform_sensors)
        self.sub_state = rospy.Subscriber(topic_root + "/platform/state",
                platform_state, self.callback_platform_state)
        self.sub_mics = rospy.Subscriber(topic_root + "/platform/mics",
                platform_mics, self.callback_platform_mics)
        self.sub_core_state = rospy.Subscriber(topic_root + "/core/state",
                core_state, self.callback_core_state)

	self.sub_pid_control = rospy.Subscriber("/control_effort", Float64, self.callback_pid_control)

        
        # set active
        self.active = True

if __name__ == "__main__":
    rospy.init_node("miro_ros_client_with_parameters", anonymous=True)
    main = miro_ros_client()
    main.loop()
