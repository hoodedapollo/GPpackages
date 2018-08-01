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

class OdometryEvaluator:

    def callback_miro_vel(self,object):
            self.v = object.odometry.twist.twist.linear.x
            self.w = object.odometry.twist.twist.angular.z

	
            self.current_time = rospy.Time.now()
                
            #compute odometry of a 2,0 robot given the linear and angular velocities
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = self.v * cos(self.th) * dt / 1000 # from mm to m
            delta_y = self.v * sin(self.th) * dt / 1000 # from mm to m
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


    def callback_new_obstacle(self, object):
            self.new_obstacle = object.data

            if self.new_obstacle:
                    self.x = 0
                    self.y = 0
                    self.th = 0
                    self.new_obstacle = False # avoid to init multiple times due to bad message synch

    def loop(self):
#        rate = rospy.Rate(50.0)	
        while not rospy.is_shutdown():
		rospy.spin()


#            rate.sleep();

    def __init__(self):

	# report
        print("initialising...")
        print(sys.version)

	# topic root
	self.robot_name = rospy.get_param('robot_name') # sim01 for simulation rob01 for real miro
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
