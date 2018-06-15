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
        # Recommended Int16 adn Float32
        self.pub = rospy.Publisher('hearbeat', Float32, queue_size=10)
        
        #shift for moving to the hour of maximum Rate
        # the 360/6 values are beacuse the maximum is at 6
        
        maximunRateHour = 12
        self.maxCircadiamRateMinutes = (maximunRateHour * 60) - 360
        self.maxCircadiamRateHours = (maximunRateHour) - 6
        
        
        #The maximum amplitude that we want to apply to the sinusoid and the average
        self.average = 21
        self.max_amp = 20
        
        #For checking the noise
        
        rospy.Timer(rospy.Duration(10), self.noiseCallback)
        self.noiseModificator = 0
        
        
        self.rate = rospy.Rate(0.2) # 0.2hz (every 5 seconds)
        
        initial_status = "starting and running...."
        rospy.loginfo(initial_status)
        self.start()



    def noiseCallback(self,x):
        with open('/tmp/meter.log', 'r') as f:
            lines = f.read().splitlines()
            last_line = lines[-1]
            rospy.loginfo(last_line)
            # chain contains ['date', 'time', 'RMS']

            chain = last_line.split()
            
            # This should be modified for a patter over some degree of noise
            self.noiseModificator = int(chain[2]) / 100

    
    
    def heartbeatRythmHours(self,x):
        total =  (self.max_amp*np.sin((2*np.pi/24) * (x - self.maxCircadiamRateHours))+self.average ) + self.noiseModificator
        return total 

    def heartbeatRythmMinutes(self,x):
        total =  (self.max_amp*np.sin((2*np.pi/1440) * (x - self.maxCircadiamRateMinutes))+self.average ) + self.noiseModificator
        return total
    
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