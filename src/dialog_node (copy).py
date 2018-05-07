#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Bool

import constants as cst
import contextlib

import json
import string
import pyttsx
import features
import inspect
import time
import sys
import signal



class KeywordsHandler():
    def __init__(self, client, callback, keywords, treshold):
        self.client = client
        self.callback = callback
        self.keywords = keywords
        self.treshold = treshold

class Dialog():

    def __init__(self):

        rospy.init_node('dialog', anonymous=True)
        time.sleep(1)

        self.clients = {}
        self.pub = rospy.Publisher('speaker', String, queue_size=10)
        self.pub_2 = rospy.Publisher('speaking', Bool, queue_size=10)

        self.intents = self.load_json(cst.INTENTS_JSON)
        self.actions = self.load_json(cst.ACTIONS_JSON)
        self.sentences = self.load_json(cst.SENTENCES_JSON)
        self.punctuation = '!"#$%&()*+,-./:;<=>?@[\]^_`{|}~'

        """for name, obj in inspect.getmembers(features):
            if inspect.isclass(obj) and issubclass(obj, features.Client):
                print obj
                print obj(self)
                print 'next'
        """

        self.classes = [obj(self) for name, obj in inspect.getmembers(features) if inspect.isclass(obj) and issubclass(obj, features.Client)]



        self.last_answer = 'nothing'
        self.last_sentence = 'nothing'
        self.hot_topic = None

        for c in self.classes:
            c.do_subscribe()

        signal.signal(signal.SIGINT, self.on_exit)

        #self.process('can you set a timer for one minute and one hundred and two seconds please')
        self.process('say a joke')
        self.listener()


    def on_exit(self, signal,frame):
        """
        Method called on a SIGINT to exit running threads before exiting that could have beend called at runtime
        """
        for c in self.classes:
            #print 'Exitting ' + str(c) + ' ...'
            c.cancel()
        exit(0)

    def load_json(self, path):
        with contextlib.closing(open(path)) as json_data:
            return json.loads(json_data.read())

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' I heard {}, trying to process the sentence ...'.format(data.data))
        
        
        self.process(data.data)
        self.last_sentence = data.data

    def listener(self):
        rospy.Subscriber("transcription", String, self.callback)
        rospy.spin()


    def pre_processing(self, data):
         #Apply a pre-processing in order to understand the data
        data = data.lower() #To lower case to match the pattern
        data = data.strip(' \t\n\r') #Remove useless spaces, tabulations and new lines. Just in case
        data = data.translate(None, self.punctuation) #Remove any poncutation to match the pattern

        print ""
        rospy.loginfo(rospy.get_caller_id() + ' Data after pre-processing :' + data)

        return data

    def remove_part(self, sentence, part, index):
        array = sentence.split()
        if index >= len(array):
            return
        if array[index] == part:
                del array[index]

                return ' '.join(array)
        return sentence

    def get_best_client(self, clients, data):

        #print clients

        relevance = {client:0 for client in clients}

        for client, patterns in clients.iteritems():
            for keyword, weight in patterns.iteritems():

                if keyword in data:
                    relevance[client] += weight
                elif keyword == '*':
                    relevance[client] = sys.maxint
        
        #best = max(relevance, key=lambda key: relevance[key])
        #print relevance

        best_score = -sys.maxint-1
        best_client = None
        for client, score in relevance.iteritems():
            if score > best_score and score >= client.treshold:
                best_score = score
                best_client = client

        #print relevance

        return best_client

    def process(self, data):

        data = self.pre_processing(data)
        #Save the data in case of it's not a pre-programmed sentence
        old_data = data

        
        sentence = self.sentences['sentences'].get(data)

        #If there is not match, remove courtesy and try again
        if sentence is None:
            
            data = self.remove_part(data, 'you', 1)
            data = self.remove_part(data, 'can', 0)
            sentence = self.sentences['sentences'].get(data)

            if sentence is None:

                data = self.remove_part(data, 'please', -1)
                sentence = self.sentences['sentences'].get(data)

        
        if sentence is None:
            rospy.loginfo('Sentence not in the static vocabulary ...')
            data = old_data

            #Try to parse the sentence to retrieve keywords from the hot topic
            if self.hot_topic is not None:     
                client = self.get_best_client(self.hot_topic, data)
                if client is not None:
                    #Reset the hot topic
                    self.hot_topic = None
                    self.publish_data(client.callback(data))
                    return
                rospy.loginfo('The sentence does not match the hot topic ...')

            #Try to parse the sentence to retrieve keywords if there is clients
            if self.clients:
                client = self.get_best_client(self.clients, data)
                if client is not None:
                    self.publish_data(client.callback(data))
                    return
                rospy.loginfo('The sentence does not match any passive keyword ...')

        
        """if sentence is None and self.clients:
            data = old_data
            client = self.get_best_client(self.clients)
            if client is not None:
                self.publish_data(client.callback(data))
                return"""

        """
        #Try to parse the sentence to retrieve keywords if there is clients
        if sentence is None and self.clients:
            data = old_data
            relevance = {client:0 for client in self.clients}
            print self.clients

            for client, patterns in self.clients.iteritems():
                for keyword, weight in patterns.iteritems():

                    if keyword in data:
                        relevance[client] += weight
                    elif keyword == '*':
                        relevance[client] = sys.maxint
            
            best = max(relevance, key=lambda key: relevance[key])
            print relevance

            if relevance[best] != 0:
                self.publish_data(best.callback(data))
                return
        """


        if sentence is None:
            rospy.logwarn('Did not understand ...')
            return


        sentence = str(sentence)
        rospy.loginfo('Understood {} from the static vocabulary'.format(sentence))

        intent = self.intents['intents'].get(sentence)

        if sentence is None:
            rospy.logerror(rospy.get_caller_id() + ' There is no intent to match to this sentence ...')
            return

        for action in intent["actions"]:
            action = str(action)
            tmp = self.actions['actions'].get(action)

            if sentence is None:
                rospy.logerror(rospy.get_caller_id() + ' There is no action match to this intent ...')
                return

            #TODO: Do smth with the action

            rospy.loginfo(rospy.get_caller_id() + 'Displaying face :' + str(tmp['face']))
            rospy.loginfo(rospy.get_caller_id() + 'Displaying gesture :' + str(tmp['gesture']))
            rospy.loginfo(rospy.get_caller_id() + 'Displaying voice :' + str(tmp['voice']))

        rospy.loginfo(rospy.get_caller_id() + ' QT says :' + intent["answer"])

        text = intent["answer"]

        if text[0] == '$':
            tmp = text[1:].split('~')

            for c in self.classes:
                if c.__class__.__name__ == tmp[0]:
                    try:
                        text = getattr(c, tmp[1])()
                        #print '###############################################'
                        return
                    except AttributeError:
                        rospy.logfatal(rospy.get_caller_id() + ' The class "{}" has no method named "{}" !'.format(tmp[0], tmp[1]))
                        return

            if text == intent["answer"]:
                rospy.logfatal(rospy.get_caller_id() + ' The class "{}" does not exists in the module "features" !'.format(tmp[0], tmp[1]))
                return

        
        self.publish_data(text)

    def subscribe(self, data, client, keywords):

        self.hot_topic = {client: keywords}

        #self.add_client(client, keywords)
        if data != '':    
            self.publish_data(data)

    def publish_data(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' Publishing {}'.format(data))

        self.last_answer = data
        self.pub_2.publish(True)
        self.pub.publish(data)

    def add_client(self, client, keywords):
        if self.clients.get(client) is not None:
            self.clients[client].update(keywords)
        else:
            self.clients[client] = keywords

if __name__ == '__main__':
    Dialog()

