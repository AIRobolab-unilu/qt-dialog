#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import String, Bool

import constants as cst
import contextlib

import json
import string
import pyttsx
import features
import calendar
import time

from time import sleep
from threading import Thread

def load_json(path):
        with contextlib.closing(open(path)) as json_data:
            return json.loads(json_data.read())

intents = load_json(cst.INTENTS_JSON)
actions = load_json(cst.ACTIONS_JSON)
sentences = load_json(cst.SENTENCES_JSON)

class TestDialog():

    def __init__(self):

        rospy.init_node('tts', anonymous=True)

        self.pub = rospy.Publisher('transcription', String, queue_size=10)
        self.reset_pub = rospy.Publisher('reset', Bool, queue_size=10)
        self.sub = rospy.Subscriber("speaker", String, self.callback)

        self.timeout = 2

        Thread(target=self.run).start()

        self.listener()

    def wait(self):


        while self.sub.get_num_connections() == 0:
            print 'waiting ...'
            rospy.sleep(0.2)
        rospy.sleep(0.2)

    def wait_until_timeout(self):
        self.result = None
        now = calendar.timegm(time.gmtime())

        while calendar.timegm(time.gmtime()) - now < self.timeout and self.result is None:
            rospy.sleep(0.2)
        return

    def run(self):
        self.wait()

        methods = [name for name in dir(self) if callable(getattr(self, name))]

        #methods = ['test_set_timer_second']

        counter = 0
        maxi = 0
        for name in methods:
            if name[:4] == 'test':
                maxi += 1
                print name
                good = True
                self.reset_pub.publish(True)
                for result, expected in getattr(self, name)():

                    #result is None if timeout or did not understand
                    if result != expected:
                        rospy.logerr('assert failed {} != {}'.format(result, expected))
                        good = False
                        break
                if good:
                    counter += 1

        rospy.loginfo('{} pass'.format(counter))
        rospy.loginfo('{} fail'.format(maxi-counter))
        rospy.loginfo(str(counter/maxi*100) + '% are ok')
        #print self.test_name()

    def test_none(self):
        possibilities = ['gbrhsg', "ble bel sd"]

        for p in possibilities:
            self.send(p)
            self.wait_until_timeout()

            yield self.result, None

    def test_name(self):
        possibilities = ['what is your name', "what's your name"]

        for p in possibilities:
            self.send(p)
            self.wait_until_timeout()

            yield self.result, intents['intents']['0']['answer']

    def test_heard(self):
        possibilities = ['what is your name', "what's your name", 'what are you looking for']

        questions = ['what did i say', 'what did you hear']
        for q in questions:
            for p in possibilities:
                self.send(p)
                self.wait_until_timeout()

                self.send(q)
                self.wait_until_timeout()

                yield self.result, 'you said '+p

    def test_heard_nothing(self):
       
        questions = ['what did i say']
        for q in questions:

            self.send(q)
            self.wait_until_timeout()

            yield self.result, 'you said nothing'

    def test_create_timer(self):
       
        questions = ['create a timer', 'would you set a timer please']
        for q in questions:

            self.send(q)
            self.wait_until_timeout()



            yield self.result, cst.ASK_TIME
            self.reset_pub.publish(True)

    def test_set_timer(self):
       
        questions = [(0, 12, 100), (1, 12, 0), (0, 0, 10)]
        for q in questions:

            self.send('set a timer for {} hour {} minutes and {} seconds'.
                format(q[0], q[1], q[2]))
            self.wait_until_timeout()


            yield self.result, cst.TIMER_SET.format(q[0], q[1], q[2])
            self.reset_pub.publish(True)

    def test_set_timer_second(self):
       
        questions = [10, 5]
        for q in questions:

            self.send('set a timer for {} seconds'.format(q))
            self.wait_until_timeout()


            yield self.result, cst.TIMER_SET.format(0, 0, q)
            self.reset_pub.publish(True)

    def test_create_list(self):
       
        questions = ['create a list', 'set a list']
        for q in questions:

            self.send(q)
            self.wait_until_timeout()



            yield self.result, cst.ASK_NAME
            self.reset_pub.publish(True)

    def test_name_after_list(self):
       
        questions = ['cats and dogs', 'horses']
        for q in questions:

            self.send('create a list named {}'.format(q))
            self.wait_until_timeout()



            yield self.result, cst.CREATE_LIST_1.format(q)
            self.reset_pub.publish(True)

    def test_name_before_list(self):
       
        questions = ['shopping', 'to do']
        for q in questions:

            self.send('create a {} list'.format(q))
            self.wait_until_timeout()



            yield self.result, cst.CREATE_LIST_1.format(q)
            self.reset_pub.publish(True)

    def test_create_ask_list(self):
       
        questions = ['to do', 'garbage']
        for q in questions:

            self.send('create a list')
            self.wait_until_timeout()

            self.send(q)
            self.wait_until_timeout()



            yield self.result, cst.CREATE_LIST_1.format(q)
            self.reset_pub.publish(True)
            


    def callback(self, data):
        rospy.logwarn(rospy.get_caller_id() + 'I heard {}'.format(data.data))
        
        self.speak(data.data)

    def listener(self):
        
        rospy.loginfo(rospy.get_caller_id() + ' Listening ...')
        rospy.spin()

    def send(self, data):
        rospy.logwarn(rospy.get_caller_id() + ' SENT : {}'.format(data))
        self.pub.publish(data)

    def speak(self, data):

        self.result = data

        data = data.split('|')
        
            

        #self.pub.publish(False)

if __name__ == '__main__':
    TestDialog()