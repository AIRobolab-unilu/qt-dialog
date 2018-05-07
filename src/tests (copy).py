#!/usr/bin/env python

import unittest

import contextlib
import json
import constants as cst
import calendar
import time

import rospy
from std_msgs.msg import String
from threading import Thread


PKG = 'social_robotics'

import sys
   


def load_json(path):
        with contextlib.closing(open(path)) as json_data:
            return json.loads(json_data.read())

intents = load_json(cst.INTENTS_JSON)
actions = load_json(cst.ACTIONS_JSON)
sentences = load_json(cst.SENTENCES_JSON)

#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool

import constants as cst
import contextlib

import json
import string
import pyttsx
import features

from time import sleep

class TestStaticVocabulary(unittest.TestCase):
    def __init__(self, *args, **kwargs):

        
        rospy.init_node("test_dialog")
        rospy.loginfo('start')
                
        self.pub = rospy.Publisher("transcription", String, queue_size=10)

        rospy.sleep(2)

        self.timeout = 8
        self.result = None
        #self.wait()

        

        self.listener()

    def run(self):
        print self.test_name()

    def wait(self):
        while self.sub.get_num_connections() == 0:
            print 'waiting ...'
            rospy.sleep(0.2)

    def listener(self):
        rospy.Subscriber("speaker", String, self.callback)
        rospy.loginfo(rospy.get_caller_id() + ' Listening ...')

        rospy.sleep(2)

        Thread(target = self.run).start()

        rospy.spin()

    def test_name(self):

        

        #rospy.loginfo(rospy.get_caller_id() + ' re re')
        self.send('what time is it')

        now = calendar.timegm(time.gmtime())
        while calendar.timegm(time.gmtime()) - now < self.timeout:
            if self.result != None:
                return self.result, intents['intents']['0']['answer']
            time.sleep(0.2)

        return self.result, intents['intents']['0']['answer']
        #self.assertEquals(1, 2, 'timeout '+self.result)

    def callback(self, data):
        rospy.logwarn('callback '+data)
        self.result = data.data
        #self.assertEquals(self.result, intents['intents']['0']['answer'])

    def send(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' send '+data)

        self.pub.publish(data)

if __name__ == '__main__':
    TestStaticVocabulary()
    

    