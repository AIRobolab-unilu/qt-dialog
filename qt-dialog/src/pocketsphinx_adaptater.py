#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import os
import wave
import contextlib

import constants as cst


class PSAdaptater():

    def __init__(self):

        self.pub = rospy.Publisher('detection', Bool, queue_size=10)

        rospy.init_node('vad', anonymous=True)

        #Create a thred because we need to listen to the "speaking" topic
        self.tool = VADWebRTCV2(self) #Set itself as the observer
        self.tool.start() #Start the thread

        self.speaking = False
        self.listener()       

    def listener(self):
        rospy.Subscriber("speaking", Bool, self.callback)
        rospy.spin()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}'.format(data.data))
        
        self.speaking = data.data
        self.tool.speaking = self.speaking


    def notify(self, data, sample_rate) :
        """Writes a .wav file.
        Takes path, PCM audio data, and sample rate.
        """

        if self.speaking:
            return

        with contextlib.closing(wave.open(cst.OUTPUT_FILE, 'wb')) as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(data)
            self.pub.publish(True)

if __name__ == '__main__':
    try:
        PSAdaptater()
    except rospy.ROSInterruptException:
        pass
