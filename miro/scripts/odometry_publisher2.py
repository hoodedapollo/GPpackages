#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose, Point

import std_msgs
from std_msgs.msg import Bool

import miro_msgs
from miro_msgs.msg import platform_sensors

import nav_msgs
from nav_msgs.msg import Odometry  	

import tf

import math
from math import sin, cos, pi
import numpy
import time
import sys

################################################################

def usage():
    print """
Usage:
    miro_ros_client.py robot=<robot_name>

    Without arguments, this help page is displayed. To run the
    client you must specify at least the option "robot".

Options:
    robot=<robot_name>
        specify the name of the miro robot to connect to,
        which forms the ros base topic "/miro/<robot_name>".
        there is no default, this argument must be specified.
    """
    sys.exit(0)

################################################################

class OdometryEvaluator:

    def callback_miro_vel(self,object):
        self.v = object.odometry.linear.x
        self.w = object.odometry.angular.z

    def callback_new_obstacle(self, object):
        self.flag = object.data

    def loop(self):
#        rate = rospy.Rate(50.0)	
        while not rospy.is_shutdown():

            if self.new_obstacle:
                    self.x = 0
                    self.y = 0
                    self.th = 0
#                    self.new_obstacle = False

            self.current_time = rospy.Time.now()
                
            #compute odometry of a 2,0 robot given the linear and angular velocities
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = self.v * cos(self.th) * dt
            delta_y = self.v * sin(self.th) * dt
            delta_th = self.w * dt

            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

             # since all odometry is 6DOF we'll need a quaternion created from yaw
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # first, we'll publish the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "base_link",
                "odom")
                
            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_link"
                #  odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

            # publish the message
            self.odom_pub.publish(odom)

            self.last_time = self.current_time;

#            rate.sleep();

    def __init__(self):

	# report
        print("initialising...")
        print(sys.version)

        # no arguments gives usage
        if len(sys.argv) == 1:
            usage()

        # options
        self.robot_name = ""
        self.drive_pattern = ""

        # handle args
        for arg in sys.argv[1:]:
            f = arg.find('=')
            if f == -1:
                key = arg
                val = ""
            else:
                key = arg[:f]
                val = arg[f+1:]
            if key == "robot":
                self.robot_name = val
            else:
                error("argument not recognised \"" + arg + "\"")

        # check we got at least one
        if len(self.robot_name) == 0:
            error("argument \"robot\" must be specified")

	# topic root
        topic_root = "/miro/" + self.robot_name
        print "topic_root", topic_root

        
        # attribute init
        self.x = 0
        self.y = 0
        self.th = 0

        self.v = 0
        self.w = 0

        self.new_obstacle = True

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()	

        # subscribers
        self.platform_semsors_sub = rospy.Subscriber(topic_root + '/platform/sensors', 
                platform_sensors, self.callback_miro_vel, queue_size = 1)
        self.new_obstacle_sub = rospy.Subscriber('/new_obstacle', 
                Bool, self.callback_new_obstacle, queue_size = 1)	

        # publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size = 0)	

        # broadcasters
        self.odom_broadcaster = tf.TransformBroadcaster()
        

if __name__ == '__main__':
    rospy.init_node('odometry_publisher2', anonymous=True)
    main = OdometryEvaluator()
    main.loop()
