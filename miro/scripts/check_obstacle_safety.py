#!/usr/bin/env python

import rospy

import std_msgs
from std_msgs.msg import Bool

import miro_msgs
from miro_msgs.msg import platform_sensors

class ObstacleSafetyChecker():
    def callback_sonar_distance(self, object):
        self.sonar_distance = object.sonar_range.range
        if ( 0.03 < self.sonar_distance < self.safety_threshold):
            self.safe = False
        else:
            self.safe = True
        safePub.pub.publish(safe)

    def loop(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def __init__(self):
        # attributes initialization
        self.sonar_distance = 0
        self.safety_threshold = 0.70

        # subscribers
        rospy.Subscriber('miro/rob01/platform/sensors', platform_sensors, self.callback_sonar_distance, queue_size = 1)

        # publishers            .
        self.safe_pub = rospy.Publisher('safe', Bool, queue_size = 0)

if __name__ == '__main__':
    rospy.init_node('check_obstacle_safety', anonymous = True)
    main = ObstacleSafetyChecker()
    main.loop()
