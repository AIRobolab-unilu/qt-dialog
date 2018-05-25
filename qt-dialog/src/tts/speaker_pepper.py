#!/usr/bin/env python

import rospy

from std_msgs.msg import String

class PepperSpeech():

    def __init__(self):
        self.pub = rospy.Publisher("speech", String, queue_size=10)

    def say(self, arg):
    
        self.pub.publish(arg)
        rospy.sleep(len(arg)*0.25)

if __name__ == '__main__':
    pass
