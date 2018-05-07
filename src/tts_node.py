#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool

import constants as cst
import contextlib

import json
import string
import pyttsx
import features

from tts.speaker_pyttsx import PyTTSx
from tts.speaker_pico import Pico

from time import sleep

class TTS():

    def __init__(self, tool):

        rospy.init_node('tts', anonymous=True)

        self.pub = rospy.Publisher('speaking', Bool, queue_size=10)
        self.tool = tool

        self.listener()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}, trying to say the sentence ...'.format(data.data))
        
        self.speak(data.data)

    def listener(self):
        rospy.Subscriber("speaker", String, self.callback)
        rospy.loginfo(rospy.get_caller_id() + ' Listening ...')
        rospy.spin()

    def init_and_say(self, text):
        self.engine = pyttsx.init()
        self.engine.setProperty('rate', 150)
        self.engine.say(text)
        self.engine.runAndWait()

    

    def speak(self, data):

        data = data.split('|')
        



        print data

        for text in data:

            self.tool.say(text)
            

        self.pub.publish(False)

if __name__ == '__main__':
    #TTS(PyTTSx())
    TTS(Pico())