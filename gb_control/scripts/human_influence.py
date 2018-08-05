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
        self.obstacle_avoidance=False
        self.lastyaw=False
        self.influence_counter=0
        self.human_influence=0
        self.human_habit_gain=1
        
        self.rate=rospy.get_param('rate',200)

        self.subSmartwatchObstacle = rospy.Subscriber('/synch_oa_flag',Bool,self.callbackObstacleAvoidance,queue_size=1)
        self.pub_human_influence = rospy.Publisher ( '/human_influence',Int64,queue_size=0)
        self.HumanTendencyEvaluation()



    def callbackObstacleAvoidance ( self, obstacle):

        self.obstacle_avoidance=obstacle.data
        

    def HumanTendencyEvaluation (self):

        r=rospy.Rate(self.rate)
        while not rospy.is_shutdown():


            if self.obstacle_avoidance:

                if self.g.gesture > 1.5:

                    self.lastyaw=True

                else:

                    if self.lastyaw:

                        self.influence_counter=self.influence_counter+1

                        self.lastyaw=False

                        rospy.loginfo("counting.......")


            if not self.obstacle_avoidance:

                if self.influence_counter > 2:

                #keeps into account how many times the human tries to escape the obstacle avoidance behaviour
                    self.human_habit_gain=self.human_habit_gain+1

                self.influence_counter=0


            self.human_influence=self.human_habit_gain*self.influence_counter

            self.pub_human_influence.publish(self.human_influence)

            rospy.loginfo("influence counter: %s, human habit gain: %s, obstacle_avoidance: %s, gyro vel: %s", self.influence_counter, self.human_habit_gain , self.obstacle_avoidance, self.g.gesture )
            
            r.sleep()

if __name__== '__main__':

    rospy.init_node('human_influence') 
    hi = HumanIntention()
    hi.HumanTendencyEvaluation()

    #mb.main()












