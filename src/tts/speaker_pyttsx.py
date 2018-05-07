#!/usr/bin/env python
# license removed for brevity
import rospy
import pyttsx

class PyTTSx():

    def __init__(self):
        pass

    def say(self, arg):
    
        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate-5)
        engine.say(arg)
        engine.say("   ")
        engine.runAndWait()

if __name__ == '__main__':
    pass
