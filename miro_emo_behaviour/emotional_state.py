#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float32

from miro_msgs.msg import core_state

#In this block we are going to establish the weight that the emotionality of MiRo give.
#We receive MiRo humor condition (mood and sleep) mapp them pubishing  a float in a range of [0,1]
#This value is the weight that influence others behaviours (multiplication with factor of gb,ob,os) because the idea of MiRo as a pet companion is based on the willgness of miro to follow others behaviours


#If the value of sleep is [0.0,1.0]  emotional state wil be 0 so no behavior is allowed
#We have an MiRo message as output

class EmotionalState:
	def __init__(self):
		self.valence = 1.0
		self.arousal = 0.5
		self.wakefulness = 1.0
		self.pressure = 0.0
		self.emotional_state = 1.0
		# subscribe
		self.sub_emotional_affect = rospy.Subscriber('/miro/core/state', core_state, self.callback_core_affect,queue_size = 1)
		#publisher
		self.pub_emotional_state = rospy.Publisher('/emotional_state', Float32, queue_size = 0)

	def callback_core_affect(self,data_affect):
	
		self.valence = data_affect.mood.valence
		self.arousal = data_affect.mood.arousal
		self.wakefulness = data_affect.sleep.wakefulness
		self.pressure = data_affect.sleep.pressure

		#For every condition there is the condintion on sleep because if miro is sleeping emotional state will be zero
		if ( self.valence == 1.0 and self.arousal == 0.5): #HAPPY

			if(self.wakefulness == 1.0  and self.pressure == 0.0):
			
				self.emotional_state = 1.0
				print"HAPPY"

			elif (self.wakefulness == 0.0 and self.pressure == 1.0):

				self.emotional_state = 0.0
				print"I'M DOING SWEET DREAMS NOW"

		elif (self.valence == 0.0 and self.arousal == 0.5):

			if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.30
					print"UNHAPPY"

			elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"STOP BOTHERING ME, I'M SLEEPING"
			

		elif (self.valence == 1.0 and self.arousal == 1.0):

			if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.80
					print"EXCITED"

			elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I WOULD LIKE TO WAKE UP AND PLAY WITH YOU BUT I'M SLEEPING"

		elif (self.valence == 0.5 and self.arousal == 1.0):

				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.4
					print"ACTIVE"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I'M SLEEPING"

		
		elif (self.valence == 0.0 and self.arousal == 1.0):

				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.1
					print"ANGRY"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"STOP BOTHERING ME, I'M SLEEPING"

		elif (self.valence == 1.0 and self.arousal == 0.0):

				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.75
					print"SERENE"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I'M DOING SWEET DREAMS NOW"

		
		elif (self.valence == 0.0  and self.arousal == 0.0):

				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.20
					print"BORED"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I WANT SLEEP"

		
		elif (self.valence == 0.5 and self.arousal == 0.0):

				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.5
					print"RELAXED"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I'WOULD LIKE TO SLEEP"

		elif (self.valence == 0.5 and self.arousal == 0.5 ):
		
				if(self.wakefulness == 1.0  and self.pressure == 0.0):
				
					self.emotional_state = 0.60
					print"NEUTRAL"

				elif(self.wakefulness == 0.0 and self.pressure == 1.0):

					self.emotional_state = 0.0
					print"I'M SLEEPING NOW"
		self.pub_emotional_state.publish(self.emotional_state)

		print(self.emotional_state)

	def main(self):
		rospy.spin()


if __name__ == "__main__":
	rospy.init_node("EmotionalState_py", anonymous=True)
	mains = EmotionalState()
	mains.main()
