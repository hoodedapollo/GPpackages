#!/usr/bin/env python

################################################################

import rospy
from std_msgs.msg import String,Bool,Float32, Int64
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

	    
		self.obstacle_avoidance=False
		self.smartwatch_state=False
		self.subObstacleAvoidance = rospy.Subscriber('/synch_oa_flag',Bool,self.callbackObstacleAvoidance,queue_size=1)
		self.subSwState = rospy.Subscriber('is_publishing',Bool,self.callbackSwState,queue_size=1)


	def callbackObstacleAvoidance(self,obstacle):

		self.obstacle_avoidance=obstacle.data

	def callbackSwState(self, smartwatch):

		self.smartwatch_state = smartwatch.data

class Gain():

	def __init__(self):

		self.human_influence = 0.0
		self.m_emotion = 0.0 
		self.subHumanInfluence = rospy.Subscriber ( '/human_influence',Float32,self.callbackHumanInfluence,queue_size=1)
		self.subEmotion = rospy.Subscriber ('/emotional_state',Float32,self.callbackEmotion,queue_size=1)

	def callbackHumanInfluence(self, influence):
		
		self.human_influence=influence.data


	def callbackEmotion(self,emotion):

		self.m_emotion = emotion.data

class Behavior():

	def __init__(self):
        
                #Emotional Behavior
                self.q_e = platform_control()

                #Obstacle Safety Behavior
                self.q_os = platform_control()
        
                #Obstacle Avoidance Behavior
	        self.q_oa = platform_control()

                #Gesture Based Behavior
		self.q_gb = platform_control()
                self.q_gb.body_config = [0.0,0.0,0.0,0.0]
                
                self.subEB = rospy.Subscriber('/emotional_behaviour',platform_control, self.callbackEB,queue_size=1)
                self.subOSB = rospy.Subscriber('/obstacle_safety_behaviour',platform_control, self.callbackOSB,queue_size=1)
		self.subGBB = rospy.Subscriber('/gesture_based_behaviour',Twist, self.callbackGBB,queue_size=1)
		self.subOAB = rospy.Subscriber('/obstacle_avoidance_behaviour',platform_control, self.callbackOAB,queue_size=1)

        def callbackEB ( self, emotional_reaction):
		
		self.q_e.lights_raw = emotional_reaction.lights_raw
		self.q_e.ear_rotate = emotional_reaction.ear_rotate
		self.q_e.tail = self.tail = emotional_reaction.tail
		self.q_e.eyelid_closure = emotional_reaction.eyelid_closure

	def callbackOSB(self,twist_os):

		self.q_os.body_vel.linear.x = twist_os.body_vel.linear.x
        	self.q_os.body_vel.angular.z = twist_os.body_vel.angular.z
               
                self.q_os.lights_raw = self.q_e.lights_raw


	def callbackGBB( self, twist_gb):
    
		self.q_gb.body_vel.linear.x = twist_gb.linear.x
		self.q_gb.body_vel.angular.z = twist_gb.angular.z

                self.q_gb.lights_raw = self.q_e.lights_raw 

    
	def callbackOAB(self, twist_oa):
        
		self.q_oa.body_vel.linear.x = twist_oa.body_vel.linear.x 
		self.q_oa.body_vel.angular.z = twist_oa.body_vel.angular.z
		self.q_oa.body_config = twist_oa.body_config
		self.q_oa.body_config_speed = twist_oa.body_config_speed
	        
                self.q_oa.lights_raw = self.q_e.lights_raw 


class MultipleBehavior():

	def __init__(self):

		self.r = Releaser()
		self.g = Gain()
		self.b = Behavior()
		

                # node rate
                self.rate = rospy.get_param('rate')

		#define Publisher
		self.pub_platform_control = rospy.Publisher('/miro/sim01/platform/control', platform_control, queue_size=0)
		self.pub_arrived_update = rospy.Publisher ('/arrived',Bool,queue_size=0)
                self.pub_escape = rospy.Publisher("/escape", Bool, queue_size=0)
    
	def BehaviorCoordination (self):
        
		self.body_vel=Twist()
		threshold = 24.0
		config = [0.0,0.0,0.0,0.0]
                config_speed = [0.0,0.0,0.0,0.0]
                


		q = platform_control()

                ra = rospy.Rate(self.rate)
		while not rospy.is_shutdown():

                    if self.r.smartwatch_state: 
		
			    if not self.r.obstacle_avoidance:

				    print "|GESTURE BASED|"

                                    q = self.b.q_gb

			    elif self.r.obstacle_avoidance: 
				    print "|OBSTACLE AVOIDANCE|"

                                    v_gb = self.b.q_gb.body_vel.linear.x
                                    w_gb = self.b.q_gb.body_vel.angular.z

                                    v_oa = self.b.q_oa.body_vel.linear.x
                                    w_oa = self.b.q_oa.body_vel.angular.z


				    G = self.g.human_influence / threshold
				    rospy.loginfo("b.v_gb * G %s", v_gb*G)
				    self.body_vel.linear.x=v_oa*(1-G)+(v_gb*G)
				    self.body_vel.angular.z=w_oa
				    config = self.b.q_oa.body_config
				    config_speed = self.b.q_oa.body_config_speed
                            

                                    q.body_vel = self.body_vel
                                    q.body_config=config
                                    q.body_config_speed=config_speed


				    if self.g.human_influence > threshold:
					    self.body_vel.linear.x=0.0
					    self.body_vel.angular.z=0.0                                       

					    config=[0.0,0.0,0.0,0.0]
					    config_speed=[0.0,0.0,-1,0.0]

					    q.body_vel = self.body_vel
					    q.body_config=config
					    q.body_config_speed=config_speed

					    self.pub_platform_control.publish(q)
    #                                       config_speed =[0.0,0.0,0.0,0.0]
					    rospy.sleep(1)
					    self.pub_arrived_update.publish(True)
					    self.pub_escape.publish(True)

                    else:

                            q=self.b.q_e
                            print "|EMOTION|"


			

		    self.pub_platform_control.publish(q)

		    ra.sleep()	

if __name__== '__main__':

    rospy.init_node('Coordination') 
    mb = MultipleBehavior()
    mb.BehaviorCoordination()
        
    #mb.main()

   
