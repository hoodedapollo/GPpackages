#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float32

from miro_msgs.msg import platform_control
from miro_constants import miro

#In this block we will decide the behaviour of MiRo on the basis of his humor
#If for example MiRo is happy he will move his ears or his tail and his color will be green
#the behaviour will be publish and if there aren't other behavior in place than MiRo will be free to move in the place.  

class emotional_behaviour:
	def __init__(self):
		self.ear_rotate = [0.0,0.0]
		self.eyelid_closure = 0.0
		self.lights_raw = [255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255]
		self.tail = -1.0
		self.emotional_state = 1.0
		# subscribe
		self.sub_affect = rospy.Subscriber('/emotional_state', Float32, self.callback_emotional_behaviour,queue_size = 1)
		#publisher
		self.pub_emotional_behaviour = rospy.Publisher('/emotional_behaviour',platform_control, queue_size = 0)
		self.pub_light_real = rospy.Publisher ('/miro/rob01/platform/control', platform_control, queue_size = 0)

	def callback_emotional_behaviour(self,data_affect):
		self.emotional_state = data_affect.data
		print(self.emotional_state)
			
		if(self.emotional_state == 0): #MiRo is sleeping so nothing is moving
				
			self.eyelid_closure = 0.0
			self.ear_rotate = [0.0,0.0]
			self.lights_raw = [255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255]#white color
			self.tail = -1.0
			
		else:

			if(self.emotional_state >= 0.2 and self.emotional_state < 0.4): #UNHAPPY
				q = platform_control()
				q.lights_raw = self.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0] #set his color to red
				q.tail =self.tail =  0.0
				q.ear_rotate = self.ear_rotate = [1.0,1.0]
				q.eyelid_closure = self.eyelid_closure = 0.0	
				
			
			elif(self.emotional_state < 0.2 ): #ANGRY
				q = platform_control()
				q.lights_raw = self.lights_raw = [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0]
				q.ear_rotate = self.ear_rotate = [1.0,1.0]
				q.eyelid_closure = self.eyelid_closure = 0.0
				q.tail = self.tail = 1.0
						

			elif (self.emotional_state >=0.5 and self.emotional_state < 0.7):  #RELAXED OR BORED
				q = platform_control()	
				q.lights_raw = self.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255]#the color is setted to blue
				q.eyelid_closure = self.eyelid_closure  = 0.5
				q.tail = self.tail = -1.0
				q.ear_rotate = self.ear_rotate = [0.0,0.0]
				
								
			elif(self.emotional_state > 0.3 and self.emotional_state < 0.5): #ACTIVE
				q = platform_control()
				q.lights_raw = self.lights_raw = [0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255]
				q.eyelid_closure = self.eyelid_closure = 0.0
				q.tail = self.tail = 1.0
				q.ear_rotate = self.ear_rotate = [0.0,0.0]
				
					

			elif (self.emotional_state >= 0.8): #HAPPY OR EXCITED
				q = platform_control()
				q.lights_raw = self.lights_raw = [0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0]		
				q.eyelid_closure = self.eyelid_closure = 0.0
				q.tail = self.tail = 1.0
				q.ear_rotate = self.ear_rotate = [0.0,0.0]
				

			

			elif(self.emotional_state >= 0.7 and self.emotional_state < 0.8): #SERENE
				q = platform_control()
				q.lights_raw = self.lights_raw = [0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0]
				q.eyelid_closure = self.eyelid_closure = 0.0
				q.ear_rotate = self.ear_rotate = [0.0,0.0]
				q.tail = self.tail = 0.0
			

			self.pub_emotional_behaviour.publish(q)
			qe=platform_control()
			qe.lights_raw = q.lights_raw
			self.pub_light_real.publish(qe)
						
		
	def main(self):
		rospy.spin()		
				
	
if __name__ == "__main__":
	rospy.init_node("emotional_behaviour", anonymous=True)
	mains = emotional_behaviour()	
	mains.main()		
				
		
