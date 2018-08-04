#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String, Bool, Int64
from sensor_msgs.msg import Image,CompressedImage,Range,Imu
from geometry_msgs.msg import Twist,Pose,Vector3Stamped

import math
from math import pi
import numpy
import time
import sys
from miro_constants import miro

from datetime import datetime

##DESCRIPTION OF THE NODE
#Subscribe to the /imu/datat_raw to evaluate gyro velocity along z
#Subscribe to /new obstacle to know if a new obstable has been encountered
#Subscribe to /arrived to know if Miro is inside the Obstaacle avoidance procedure
#while miro is avoiding the obstacle a human_influence gain is set based on how many time the user perform a certain gesture
#it is multiplied by a human factor gain that increase each time the user exert his influence when a new obstacle avoidance behaviour is triggered
#The resut will be that if the user try to escape from obstacle avoidance each time a new obstacle is encoutered the next time the human_influence will be higher

class GestureEvaluation():

    def __init__(self):

        self.gesture=0

        #self.subSmartwatch = rospy.Subscriber('/rpy_deg',Vector3Stamped,self.callbackrpy,queue_size=1)
        self.subSmartwatchImu = rospy.Subscriber('/imu/data_raw',Imu,self.callbackimu,queue_size=1)

    def callbackimu(self,imu):

        self.gesture=imu.angular_velocity.z

class HumanIntention():

    def __init__(self):

        self.g=GestureEvaluation()
        self.obstacle=False
        self.arrived=True
        self.lastyaw=False
        self.influence_counter=0
        self.human_influence=0
        self.human_habit_gain=0

        self.subSmartwatchobstacle = rospy.Subscriber('/new_obstacle',Bool,self.callbackNewObstacle,queue_size=1)
        self.subSmartwatcharrived = rospy.Subscriber('/arrived',Bool,self.callbackArrived,queue_size=1)
        self.pub_human_influence = rospy.Publisher ( '/human_influence',Int64,queue_size=0)
        self.HumanTendencyEvaluation()



    def callbackNewObstacle ( self, new_obstacle):

        self.obstacle=new_obstacle.data
        self.arrived = False

    def callbackArrived ( self, goal):

        self.arrived=goal.data
        self.obstacle=False

    def HumanTendencyEvaluation (self):


        if not self.arrived:

            if self.g.gesture > 1.5:

                self.lastyaw=True

            else:

                if self.lastyaw:

                    self.influence_counter=self.influence_counter+1

                    self.lastyaw=False

                    rospy.loginfo("counting.......")


        if not self.obstacle:

            if self.influence_counter > 2:

                #keeps into account how many times the human tries to escape the obstacle avoidance behaviour
                self.human_habit_gain=self.human_habit_gain+1

            self.influence_counter=0


        self.human_influence=self.human_habit_gain*self.influence_counter

        self.pub_human_influence.publish(self.human_influence)

        rospy.loginfo("influence counter: %s, human habit gain: %s, arrived: %s, obstacle: %s, gyro vel: %s", self.influence_counter, self.human_habit_gain , self.arrived, self.obstacle, self.g.gesture )

if __name__== '__main__':

    rospy.init_node('human_influence') 
    hi = HumanIntention()

    while not rospy.is_shutdown():

        hi.HumanTendencyEvaluation()

    #mb.main()












