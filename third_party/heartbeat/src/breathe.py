#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *
import datetime as dt
import numpy as np



#for modelling the heartbeat we have using a sinusoid as explained in 
#  
# Circadian rhythm of heart rate and heart rate variability, Martial M Massin, Krystel Maeyns, Nadia Withofs, Francoise Ravet, Paul Gerard
# 

class HeartBeat():

    def __init__(self):
        
        
       
        
        rospy.init_node('hearbeat_rythm', anonymous=True)
        
        #select your favourite type for the beat 
        # Recomended Int16 adn Float32
        self.pub = rospy.Publisher('hearbeat', Float32, queue_size=10)
        
        #shift for moving to the hour of maximum Rate
        # the 360/6 values are beacuse the maximum is at 6
        
        maximunRateHour = 12
        self.maxCircadiamRateMinutes = (maximunRateHour * 60) - 360
        self.maxCircadiamRateHours = (maximunRateHour) - 6
        #shift for moving to the hour of maximum Rate
        
        #shift
        
        self.rate = rospy.Rate(0.2) # 0.2hz (every 5 seconds)
        
        initial_status = "starting and running...."
        rospy.loginfo(initial_status)
        self.start()
    
    
    def heartbeatRythmHours(self,x):
        return 20*np.sin((2*np.pi/24) * (x - self.maxCircadiamRateHours))+21

    def heartbeatRythmMinutes(self,x):
        return 20*np.sin((2*np.pi/1440) * (x - self.maxCircadiamRateMinutes))+21
    
    def start(self):
    
        while not rospy.is_shutdown():
            hour = dt.datetime.now().hour
            minute = (60*hour) + dt.datetime.now().minute
            status = self.heartbeatRythmMinutes(minute)
            
            #rospy.loginfo(status)
            
            self.pub.publish(status)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        HeartBeat()
    except rospy.ROSInterruptException:
        pass