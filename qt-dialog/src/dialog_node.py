#!/usr/bin/env python
from __future__ import division

import rospy
from std_msgs.msg import String, Bool

import constants as cst
import contextlib

from random import choice
#from jsgf import PublicRule, Literal, Grammar

import json
import string
import pyttsx
import features
import inspect
import time
import sys
import signal
import JSFGTools.JSGFParser as parser
import JSFGTools.JSGFGrammar as gram
from JSFGTools.DeterministicGenerator import processRHS

class Dialog():

    def __init__(self):

        rospy.init_node('dialog', anonymous=True)
        time.sleep(1)

        self.clients = {}
        self.handlers = []
        self.pub = rospy.Publisher('speaker', String, queue_size=10)
        self.pub_emotions = rospy.Publisher('emotions/setEmotion', String, queue_size=10)
        #self.pub = rospy.Publisher('/qt_tts/say', String, queue_size=10) 
        self.pub_2 = rospy.Publisher('speaking', Bool, queue_size=10)

        rospy.loginfo(rospy.get_caller_id() + str(self.pub.get_num_connections()) + '#########')

        self.init()
        

        """for name, obj in inspect.getmembers(features):
            if inspect.isclass(obj) and issubclass(obj, features.Client):
                print obj
                print obj(self)
                print 'next'
        """

        self.classes = [obj(self) for name, obj in inspect.getmembers(features) if inspect.isclass(obj) and issubclass(obj, features.Client)]



        for c in self.classes:
            c.do_subscribe()

        signal.signal(signal.SIGINT, self.on_exit)

        #self.process('can you set a timer please')
        #self.process('twenty ')
        #self.process('twenty ')
        #self.process('twenty ')
        #self.process('create a shipping list')
        #self.process('test')
        #self.process('say a joke')
        #self.process('what is your name')
        #self.process('heard well this is a stubborn enough time off at the')



        self.listener()


    def init(self):
        self.last_answer = 'nothing'
        self.last_sentence = 'nothing'
        self.hot_topic = []
        self.intents = self.load_json(cst.INTENTS_JSON)
        self.actions = self.load_json(cst.ACTIONS_JSON)
        self.sentences = self.load_json(cst.SENTENCES_JSON)
        #self.grammar = Grammar()
        self.punctuation = '!"#$%&()*+,-./:;<=>?@[\]^_`{|}~'

        self.init_grammar()

        self.retry = 0
        self.counter = 0
        self.max_retry = 2

    def create_grammar_file_old(self, sentence):



        private_rules = []
        public_rule = []

        for s, name in self.sentences['sentences'].iteritems():
            public_rule.append('<auto_generate_{}>'.format(name))
            private_rules.append('<auto_generate_{}> = {};'.format(name, s))
            

        print '{} ({});\n{}'.format(cst.GRAMMAR_HEADER, ' | '.join(public_rule), '\n'.join(private_rules))


    def create_grammar(self, sentence):

        rule = 'public <auto_generate> = {};'.format(sentence)


        #print rule
        return rule
    def init_grammar(self):

        self.grammar = {}

        for s, name in self.sentences['sentences'].iteritems():
            lines = [self.create_grammar(s)]

            grammar = parser.getGrammarObjectFromString(lines)
            for rule in grammar.publicRules:
                expansions = processRHS(rule.rhs)
                for expansion in expansions:
                    
                    #print expansion

                    self.grammar[expansion.lower()] = name

        print self.grammar
        rospy.loginfo(rospy.get_caller_id() + ' loaded {} sentences'.format(len(self.grammar)))
        #for sentence, name in self.sentences["sentence"].iteritems():
            #rule = PublicRule(str(name), Literal(sentence))
            #self.grammar.add_rule(rule)
            
        #print grammar.compile()

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
        rospy.Subscriber("reset", Bool, self.reset_callback)
        rospy.Subscriber("transcription", String, self.callback)
        rospy.spin()

    def reset_callback(self, data):
        self.init()

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
            return sentence
        if array[index] == part:
                del array[index]

                return ' '.join(array)
        return sentence

    def get_best_handler(self, handlers, data):

        #print clients

        #relevance = {client:0 for handler in handlers}
        relevance = {}

        print handlers

        for handler in handlers:



            if handler is None:
                continue

            total_counter = 0
            required_counter = 0
            for required in handler.keywords:

                if type(required) is not tuple and type(required) is not list:
                        rospy.logwarn(rospy.get_caller_id() + ' "{}" is neither a tuple nor a list, are you sure about this ?'.format(required))
                
                found = False
                for optional in required:

                    #print 'is "'+optional+'"" in "'+ data + '"" ?'

                    if handler.extendable:
                        if optional in data or optional == '*':
                            #print 'opt : '+optional
                            found = True
                            total_counter += 1
                    else:
                        if optional in data.split() or optional == '*':
                            #print 'opt : '+optional
                            found = True
                            total_counter += 1

                if found:
                    required_counter += 1

                #if counter == 0:
                    #break

            score_opt = total_counter/len(handler.keywords)
            score_rqr = required_counter/len(handler.keywords)
            if score_rqr >= 1:
                #print counter
                
                relevance[handler] = score_opt

        print relevance

        #if relevance:

        best_score = -sys.maxint-1
        best_handler = None
        for handler, score in relevance.iteritems():
            if best_handler is None:
                if score >= best_score:
                    best_score = score
                    best_handler = handler

            elif len(handler.keywords) > len(best_handler.keywords):
                best_score = score
                best_handler = handler

            elif len(handler.keywords) == len(best_handler.keywords):
                if score >= best_score:
                    best_score = score
                    best_handler = handler
            
                    

        #best = max(relevance, key=lambda key: relevance[key])
        #else:
            #best = None
        #print relevance

        return best_handler

    def process(self, data):

        data = self.pre_processing(data)
        #Save the data in case of it's not a pre-programmed sentence
        old_data = data
        
        #sentence = self.sentences['sentences'].get(data)
        sentence = self.grammar.get(data)

        #If there is not match, remove courtesy and try again
        data = self.remove_part(data, 'cutie', 0) #Mistood for 'QT'
        data = self.remove_part(data, 'kitty', 0) #Mistood for 'QT'
        data = self.remove_part(data, 'cutie', -1) #Mistood for 'QT'
        data = self.remove_part(data, 'kitty', -1) #Mistood for 'QT'
        data = self.remove_part(data, 'T.', -1) #Mistood for 'QT'
        data = self.remove_part(data, 'Q.', -1) #Mistood for 'QT'
        data = self.remove_part(data, 'Q.', 0) #Mistood for 'QT'
        data = self.remove_part(data, 'T.', 0) #Mistood for 'QT'
        if sentence is None:
            
            data = self.remove_part(data, 'you', 1)
            data = self.remove_part(data, 'can', 0)
            #sentence = self.sentences['sentences'].get(data)
            sentence = self.grammar.get(data)

            if sentence is None:

                data = self.remove_part(data, 'please', -1)
                #sentence = self.sentences['sentences'].get(data)
                sentence = self.grammar.get(data)

        
        if sentence is None:
            rospy.loginfo('The sentence is not in the static vocabulary ...')
            data = old_data

            #Try to parse the sentence to retrieve keywords from the hot topic
            if self.hot_topic:     
                handler = self.get_best_handler(self.hot_topic, data)
                if handler is not None:
                    #Reset the hot topic
                    self.hot_topic = []
                    self.publish_data(handler.callback(data))
                    return

                self.try_again()
                rospy.loginfo('The sentence does not match the hot topic ...')
                return

            #Try to parse the sentence to retrieve keywords if there is clients

            if self.handlers:
                handler = self.get_best_handler(self.handlers, data)
                if handler is not None:
                        #self.publish_data(getattr(handler.client, handler.callback)())
                    self.publish_data(handler.callback(data))
                    #print '###############################################'
                    return
                    #except AttributeError:
                        #rospy.logfatal(rospy.get_caller_id() + ' The class has no method named "{}" !'.format(handler.callback))
                        #return
                    
                    return
                rospy.loginfo('The sentence does not match any passive keyword ...')

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

            self.publish_action(action)

            rospy.loginfo(rospy.get_caller_id() + 'Displaying face :' + str(tmp['face']))
            rospy.loginfo(rospy.get_caller_id() + 'Displaying gesture :' + str(tmp['gesture']))
            rospy.loginfo(rospy.get_caller_id() + 'Displaying voice :' + str(tmp['voice']))

        rospy.loginfo(rospy.get_caller_id() + ' QT says :' + str(intent["answer"]))

        text = intent["answer"]

        if hasattr(text, '__iter__'):
            text = choice(text)

        if text[0] == '$':
            tmp = text[1:].split('~')

            for c in self.classes:
                if c.__class__.__name__ == tmp[0]:
                    try:
                        text = getattr(c, tmp[1])()
                        #print '###############################################'
                        #return
                    except AttributeError:
                        rospy.logfatal(rospy.get_caller_id() + ' The class "{}" has no method named "{}" !'.format(tmp[0], tmp[1]))
                        return

            if text == intent["answer"]:
                rospy.logfatal(rospy.get_caller_id() + ' The class "{}" does not exists in the module "features" !'.format(tmp[0], tmp[1]))
                return

        
        self.publish_data(text)

    def subscribe_to_topic(self, handler):

        self.hot_topic.append(handler)

    

    def publish_data(self, data):

        self.counter += 1

        if data == cst.RESET:
            self.reset()

        rospy.loginfo(rospy.get_caller_id() + ' Publishing {}'.format(data))

        self.last_answer = data
        self.pub_2.publish(True)
        self.pub.publish(data)

        rospy.loginfo(rospy.get_caller_id() + ' published on {}'.format(self.pub))

    def publish_action(self, action):

        tmp = self.actions['actions'].get(action)

        self.pub_emotions.publish(tmp['face'])
        #Voice
        #Gesture
        # ...

    def try_again(self):
        self.retry += 1
        
        if self.retry == self.max_retry:
            self.publish_data("i could not understand you | let's try a different topic")
            self.retry = 0
            self.hot_topic = []
        else:
            self.publish_data('i did not get that | can you repeat please')

    def add_handler(self, keywords):

        self.handlers.append(keywords)

        """if self.clients.get(client) is not None:
            self.clients[client].update(keywords)
        else:
            self.clients[client] = keywords
        """

if __name__ == '__main__':
    Dialog()

