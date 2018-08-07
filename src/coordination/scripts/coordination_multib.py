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
from miro_msgs.msg import platform_config,platform_sensors,platform_state,platform_mics,platform_control,core_state,core_control,core_config,bridge_config,bridge_stream


from datetime import datetime

##THE GENERAL IDEA
## The following node is entitle of behaviour coordination
## Four main behaviour are allowed:
## OBSTACLE SAFETY: Miro detects the obstacle and stops(*) in order to avoid collision
## OBSTACLE AVOIDANCE: Once safe Miro starts to implement an obstacle avoidance algorithm (BUG2 symplified)*
## GESTURE BASED: When Miro safe and no Ostacle avoidance beavior has been activated Miro follows command from the smartwatch**
## BIOMIMETIC: When Miro safe and no Obstacle avoidance behavior has been activated and the smartwatch is not giving commands(**) Miro performs action based on its emotinal state
## * The Obstacle avoidance Behavior can be override by the user intendion ( IF STRONG) depending on Miro emotional state ()
## ** The Gesture Based Behavior, and in particular Miro responsiveness, depent on Miro emotinal state

##DESCRIPTION OF THE NODE


##The Coordination node receives as input:
##Behavior: different v,w value for OS, OA, and GBB and a true/false flag for BM
##Releaser: Conditions that trigger different behaviors
##Gain: Condition that influence the different Behavior

##These have been implemented as Classes with different attributes 
## whose values are obtained by subscribing different topics

##The Coordination node produces as output:
## the different Behavior's v,w and weights them with gains ( if priority is not HIGH)
## or a flag that triggers a biomimetic behavior



## Subscribe to the topic /safe_flag
## read a boolean variable safe 
## safe=TRUE 
## safe=FALSE 
## Subscribe to the topic /ObstacleSafetyBehavior
## if safe = FALSE Publish v_os,w_os on MIRO red from /ObstacleSafetyBehavior
## Subscribe to the topic /goal_flag
## read a boolean variable goal
## goal=TRUE --> GOAL PRESENT
## goal=FALSE --> GOAL NOT PRESENT
## Subscribe to the topic /GestureBasedBehavior and save v_gb, w_gb
## Subscribe to the topic /HumanInfluence that gave a int value h_weight based on a particular gesture and how many time a user performs it
## Subscribe to the topic /EmotionalState that gave a int value corresponding to an emotional_state
## Subscribe to the topic /ObstacleAvoidanceBehavior and save v_oa, w_oa
## if safe = TRUE and goal=TRUE Publish on MIRO (emotional_state*h_weight*(v_gb,w_gb))+(v_oa,w_oa) (Evaluate the gain and when strong enough switch goal from true to false)
## Subscribe to the topic /smartwatch_flag
## read a boolean variable sw_state
## sw_state = TRUE --> SMARTWATCH ACTIVE
## sw_state = FALSE --> SMARTWATCH INACTIVE
## if safe = TRUE and goal = FALSE and sw_state = TRUE Publish on MIRO (emotional_state*(v_gb,w_gb))
## if safe = TRUE and goal = FALSE and sw_state = FALSE Trigger a Biomimetic Beahavior by publishing a message bioBehavior_flag /BiomimeticBehaviour

class Releaser():

    def __init__(self):

        self.gb=True
        self.oa=False
        self.smartwatch_state=False

        self.subSafety = rospy.Subscriber('/arrived',Bool,self.callbackSafety,queue_size=1)
        self.subGoal = rospy.Subscriber('/new_obstacle',Bool,self.callbackGoal,queue_size=1)
        self.subSwState = rospy.Subscriber('/smartwatch_flag',Bool,self.callbackSwState,queue_size=1)


    def callbackSafety(self,arrived): 

        self.gb=arrived.data
	    self.oa=False

    def callbackGoal(self,new_obstacle):

        self.oa=new_obstacle.data
	    self.gb=False

    def callbackSwState(self, smartwatch):

        self.smartwatch_state = smartwatch.data

class Gain():

    def __init__(self):

        self.h_influence=0.0
        self.m_emotion=0.0

        self.subHumanInfluence = rospy.Subscriber('/human_influence',Float32, self.callbackInfluence,queue_size=1)
        self.subEmotion = rospy.Subscriber('/EmotionalState',Float32, self.callbackEmotion,queue_size=1)


    def callbackEmotion(self, emotion):

        self.m_emotion = emotion

    def callbackInfluence(self, human_weight):
        
        self.h_influence=human_weight


class Behavior():

    def __init__(self):

        #Emotional Behaviour
        self.lights = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.ears = [0.0,0.0]
        self.tail = 0.0
        self.eyes = 0.0
        
        #Obstacle Safety Behavior
        self.v_os=0.0
        self.w_os=0.0
        #Obstacle Avoidance Behavior
        self.v_oa=0.0
        self.w_oa=0.0
	    self.config=[0.0,0.0,0.0,0.0]
	    self.config_speed=[0.0,0.0,0.0,0.0]
        #Gesture Based Behavior
        self.v_gb=0.0
        self.w_gb=0.0

        self.subEB = rospy.Subscriber ( '/emotional_behaviour',platform_control,self.callbackEB,queue_size=1)
        self.subOSB = rospy.Subscriber('/obstacle_safety_behaviour',Twist, self.callbackOSB,queue_size=1)
        self.subGBB = rospy.Subscriber('/gesture_based_behaviour',Twist, self.callbackGBB,queue_size=1)
        self.subOAB = rospy.Subscriber('/obstacle_avoidance_behaviour',platform_control, self.callbackOAB,queue_size=1)

    def callbackEB ( self, emotional_reaction)

        self.lights = emotional_reaction.lights_raw
        self.ears = emotional_reaction.ear_rotate
        self.tail = emotional_reaction.tail
        self.eyes = emotional_reaction.eyelid_closure

	


    def callbackOSB(self,twist_os): 

        self.v_os=twist_os.linear.x #twist_os[0]
        self.w_os=twist_os.angular.z #twist_os[1]
        

    def callbackGBB( self, twist_gb):
    
        self.v_gb=twist_gb.linear.x
        self.w_gb=twist_gb.angular.z

    
    def callbackOAB(self, twist_oa):
        
        self.v_oa=twist_oa.body_vel.linear.x 
        self.w_oa=twist_oa.body_vel.angular.z
	    self.config=twist_oa.body_config
	    self.config_speed=twist_oa.body_config_speed
	

class MultipleBehavior():


    def __init__(self):

        self.r = Releaser()
        self.b = Behavior()
	    self.flag = True 

        #define Publisher
        self.pub_platform_control = rospy.Publisher('/miro/sim01/platform/control', platform_control, queue_size=0)

    def BehaviorCoordination (self):
        ##TO DO:
        # add the cosmetic dof to zero in the other behaviours
        # add the human influence and check topics for the emotional state and everything
        # publish the new flag arrived = true when the gain exceed the threshold
        
        self.body_vel=Twist()

	    while not rospy.is_shutdown():

            if self.smartwatch_state = True

                if self.r.gb:

                    print "|GESTURE BASED|"

                    self.body_vel.linear.x=self.b.v_gb
                    self.body_vel.angular.z=self.b.w_gb

                elif self.r.oa: 

                    print "|OBSTACLE AVOIDANCE|"
                    self.body_vel.linear.x=self.b.v_oa
                    self.body_vel.angular.z=self.b.w_oa
            
                    rospy.loginfo(self.r.gb)

            if self.smartwatch_state = False

                if obstacle # to check the flag for safety

                    print "|OBSTACLE SAFETY|"

                    self.body_vel.linear.x=b.v_os
                    self.body_vel.angular.z=b.w_os

                else:

                    print "|EMOTIONAL|"
                
        	q = platform_control()
        	q.body_vel = self.body_vel
		    q.body_config=self.b.config
		    q.body_config_speed=self.b.config_speed

        	self.pub_platform_control.publish(q)


    
    """ def main (self):
        
        rospy.spin() """
        

if __name__== '__main__':

    rospy.init_node('Coordination') 
    mb = MultipleBehavior()
    mb.BehaviorCoordination()
        
    #mb.main()

   
