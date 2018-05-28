#!/usr/bin/env python
import rospy

try:
    import pyttsx                 
except ImportError:
    print 'Could not import {}'.format('pyttsx')

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
