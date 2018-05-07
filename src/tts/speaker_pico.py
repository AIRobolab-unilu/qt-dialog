#!/usr/bin/env python
# license removed for brevity
import rospy
import pyttsx

from svox_tts.srv import Speech

class Pico():

    def __init__(self):
        self.service = rospy.ServiceProxy('/svox_tts/speech', Speech)

        self.service(2, 'test', 100)
        self.service(3, 'test', 150)

    def say(self, arg):
        self.service(5, arg, 0)
        

if __name__ == '__main__':
    pass
