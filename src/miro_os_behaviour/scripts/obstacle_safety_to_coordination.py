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

    def callback_safe(self, object):
        if not object.data:
            q = platform_control()
	    q.body_config_speed = [0,0,0,0]
	    q.body_config = [-1,-1,-1,-1]
            q.body_vel.linear.x = -100
            q.body_vel.angular.z = 0
            self.pub_platform_control.publish(q)


    def loop(self):
        rospy.spin()

    def __init__(self):
        
        # publish
        self.pub_platform_control = rospy.Publisher("/obstacle_safety_behaviour",
                    platform_control, queue_size=0)
	
        # subscribe
	self.sub_safe = rospy.Subscriber("/safe", Bool, self.callback_safe)

if __name__ == "__main__":
    rospy.init_node("obstacle_safety_to_coordination", anonymous=True)
    main = miro_ros_client()
    main.loop()
