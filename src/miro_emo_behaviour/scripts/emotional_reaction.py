#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Int32

from miro_msgs.msg import platform_sensors,core_state,platform_mics,platform_control

#The target will be to take information from sensors as touch sensors. Emotional state of miro will be influenced by these.
#We consider where is touched and how many time is touched
# I receive three signal time that represents how time MiRo have passed in the previous behaviours
#In our case there are three signals because we consider three behaviuor : obstacle avoidance, obstacle safety and gesture based
#if too much time passes so MiRo will be arousal or unhappy until he fall asleep
#We publish the MiRo humor condition (mood and sleep)
 
def fmt(x, f): #function used for conversion from byteArray to String (The values that we get for the head touch sensors are byteArrays)
    s = ""
    x = bytearray(x)
    for i in range(0, len(x)):
        if not i == 0:
            s = s + ", "
        s = s + f.format(x[i])
    return s

class EmotionalReaction:
	
    def __init__(self):
		
        #initialize the humor variables
        self.mood = [1.0,0.5]		
        #initialize the sleep variable
        self.sleep = [1.0,0.0]
        #initialize 4 counters 
        self.count1 = 0
        self.count2 = 0
        self.count3 = 0
        self.count4 = 0
       
        #initialize touch vector
        self.head = [0,0,0,0]
        self.body = [0,0,0,0]

        #initialize time variables
        self.time_oa = 0
        self.time_gb = 0
        self.time_os = 0
		
        # subscribe
        self.sub_sensors_touch = rospy.Subscriber('/miro/rob01/platform/sensors', platform_sensors, self.callback_platform_sensors,queue_size =1)
        self.sub_time = rospy.Subscriber('/miro_active_time_OS',Int32 , self.callback_os_time , queue_size = 1)
        self.sub_time = rospy.Subscriber('/miro_active_time_OA',Int32 , self.callback_oa_time , queue_size = 1)
        self.sub_time = rospy.Subscriber('/miro_active_time_GB',Int32 , self.callback_gb_time , queue_size = 1)
        
        #publisher
        self.pub_affect = rospy.Publisher('/miro/core/state', core_state, queue_size = 0)
        self.pub_color = rospy.Publisher('/miro/rob01/platform/control',platform_control, queue_size = 0)
    
    #first callback for the touch sensors
    def callback_platform_sensors(self,datasensor):

        h1 = int(fmt(datasensor.touch_head, '{0:.0f}')[0]) 
        h2 = int(fmt(datasensor.touch_head, '{0:.0f}')[3]) 
        h3 = int(fmt(datasensor.touch_head, '{0:.0f}')[6])
        h4 = int(fmt(datasensor.touch_head, '{0:.0f}')[9])
        b1 = int(fmt(datasensor.touch_body, '{0:.0f}')[0])
        b2= int(fmt(datasensor.touch_body, '{0:.0f}')[3]) 
        b3= int(fmt(datasensor.touch_body, '{0:.0f}')[6]) 
        b4= int(fmt(datasensor.touch_body, '{0:.0f}')[9])
        q = core_state()
        #h1 and h2 are sensors behind ears 
        if(h1 == 1 or h4 == 1):  
            self.count1 = 0
            self.count2 = self.count2 + 1
            self.count3 = 0
            self.count4 = 0
            if(self.count2 >= 50):
                self.mood = [0.5,0.0] #RELAXED

            else:
                self.mood = [1.0,0.5] #HAPPY
        #h2 and h3 are sensors on the front
        if(h2 == 1 or h3 == 1):
            self.count1 = self.count1 + 1
            self.count2 = 0
            self.count3 = 0
            self.count4 = 0
			
            if(self.count1 >= 100):
                self.mood = [1.0,1.0] #EXCITED
    
            else:
                self.mood = [1.0,0.0] #SERENE	
				
		#b1 and b2 are sensors along the body near the tail
        if(b1 == 1 or b2 == 1):
            self.count1 = 0
            self.count2 = 0
            self.count4 = 0
            self.count3 = self.count3 + 1

            if( self.count3 >= 100 ):
                self.mood = [0.0,1.0]#ANGRY

            else:
                self.mood = [0.0,0.5]#UNHAPPY

        #b3 and b4 are sensors along the body near head
        if(b3 == 1 or b4 == 1):
            self.count1 = 0
            self.count2 = 0
            self.count3 = 0
            self.count4 = self.count4 + 1

            if(self.count4 >= 100):
                self.mood = [0.5,1.0]#ACTIVE

            elif(self.count4 >= 50 and self.count4 < 100):
    
                self.mood = [0.5,0.5]#NEUTRAL
    
            else:
                self.mood = [0.5,0.0]#RELAXED

	
        q.mood.valence = self.mood[0]
        q.mood.arousal = self.mood[1]
        q.sleep.wakefulness = self.sleep[0]
        q.sleep.pressure = self.sleep[1]
        self.pub_affect.publish(q)
        rospy.loginfo(q.mood.valence)
        rospy.loginfo(q.mood.arousal)

	
    def callback_os_time(self, datatime) :
        self.time_os = datatime.data

    def callback_oa_time(self, datatime) :
        self.time_oa = datatime.data


    def callback_gb_time(self, datatime) :
        self.time_gb = datatime.data
    
    def time_reaction(self):
        rospy.loginfo(self.time_os)
        r = platform_control()
        #In the first and in second condition miro is sleeping
        if(self.time_gb >= 120):
            self.mood = [0.0,0.0]
            self.sleep = [0.0,1.0] #value of sleep [0.0,1.0] emotional_state will be 0 NO RESPONSE
            r.lights_raw  [255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255] #set his color to white
            self.pub_color.publish(r)
        elif(self.time_gb >= 80 and time_gb < 120 or time_oa >= 120):
            self.mood = [0.0,0.0]
            self.sleep = [0.0,1.0]
            r.lights_raw  [255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255] #set his color to white         
            self.pub_color.publish(r)
        #third condition too much time in oa or in gb miro will be angry
        elif(self. time_oa >= 60 or self.time_gb >=60):
            self.mood = [0.0,1.0] #ANGRY emotional_state must be 0.1
            self.sleep = [1.0,0.0]
            r.lights_raw  [255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0] #set his color to red
            self.pub_color.publish(r)
        elif(self.time_oa >= 30 or self.time_os >= 30):
            self.mood = [0.5,1.0] #ACTIVE emotional state must be 0.4
            self.sleeo = [1,0,0.0]
            r.lights_raw  [0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255] #set his color to blue
            self.pub_color.publish(r)
        elif(self.time_os >= 20 and self.time_os < 30 ):
            self.mood = [1.0,1.0]#EXCITED emotional state must be 0.8
            self.sleep = [1.0,0.0]
            r.lights_raw  [0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0]#set his color to green
            self.pub_color.publish(r)
        else:
            self.mood = [1.0,0.5] #HAPPY emotional state must be 1
            self.sleep = [1.0,0.0]
            r.lights_raw  [0,255,0,0,255,0,0,255,0,0,255,0,0,255,0,0,255,0]#set his color to green
            self.pub_color.publish(r)

        q = core_state()
        q.mood.valence = self.mood[0]
        q.mood.arousal = self.mood[1]
        q.sleep.wakefulness = self.sleep[0]
        q.sleep.pressure = self.sleep[1]
        self.pub_affect.publish(q)


    def main(self):
        rospy.spin()



if __name__ == "__main__":
    rospy.init_node("EmotionalReaction_py", anonymous=True)
    mains = EmotionalReaction()
    mains.main()
    while not rospy.is_shutdown():
        mains.time_reaction()
    
