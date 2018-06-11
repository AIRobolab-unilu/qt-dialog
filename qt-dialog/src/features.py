#!/usr/bin/env python
import contextlib
import constants as cst

from random import randint, choice, shuffle
from std_msgs.msg import String
from motivational_component.srv import * 

import io
import threading
import datetime
import calendar
import json
import datetime
import rospy

import time
from abc import ABCMeta, abstractmethod



CREATION = ('set', 'create', 'make')
ADD = ('add', 'append')

START = ('start', 'begin', 'have', "let's", 'let')
END = ('end', 'over', 'other', 'and', 'stop', 'quit', 'exit') #the STT can get 'other' instead of 'over'

TOPIC = ('conversation', 'discussion', 'topic', 'talk')

AFFIRMATION = ('yes', 'yeah', 'yep', 'ok', 'okay')
NEGATION = ('not', "don't", 'no', "did'nt", "'nt")

PERSONNAL = ('i', 'me', 'am', "i'm")
OTHER = ('you',)

EMOTIONS_GOOD = ('good', 'joy', 'happy', 'well', 'better', 'fine')
EMOTIONS_BAD = ('bad', 'pain', 'hurt', 'sad', 'worse')

EMOTIONS_ANGRY = ('angry', 'coleric')

pub = rospy.Publisher('qt_face/setEmotion', String, queue_size=10)



def is_negative(sentence):
    negative = False
    for word in sentence.split():
        if word in NEGATION:
            negative = not negative

    return negative

class KeywordsHandler():
    def __init__(self, callback, keywords, extendable=False):
        self.callback = callback
        self.keywords = keywords
        self.extendable = extendable

    def __repr__(self):
        return json.dumps(self.keywords )


def text2int(textnum, numwords={}):
    """
    This function transforms a text into a list which contains words and int converted from letters.
    Be carrefull, if the word 'and' is not part of a number (ig, ten cats and one dog) it will be replaced by a zero
    """

    sentence = []

    if not numwords:
      units = [
        "zero", "one", "two", "three", "four", "five", "six", "seven", "eight",
        "nine", "ten", "eleven", "twelve", "thirteen", "fourteen", "fifteen",
        "sixteen", "seventeen", "eighteen", "nineteen",
      ]

      tens = ["", "", "twenty", "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety"]

      scales = ["hundred", "thousand", "million", "billion", "trillion"]

      numwords["and"] = (1, 0)
      for idx, word in enumerate(units):    numwords[word] = (1, idx)
      for idx, word in enumerate(tens):     numwords[word] = (1, idx * 10)
      for idx, word in enumerate(scales):   numwords[word] = (10 ** (idx * 3 or 2), 0)

    current = result = 0
    found = False
    for word in textnum.split():
        if word not in numwords:
            if found:
                sentence.append(result + current)
                sentence.append(word)
            else:
                sentence.append(word)
            result = 0
            current = 0
            found = False
            continue
            #raise Exception("Illegal word: " + word)

        found = True

        scale, increment = numwords[word]
        current = current * scale + increment
        if scale > 100:
            result += current
            current = 0

    #sentence.append(result + current)

    return sentence


def file_len(fname):
    with contextlib.closing(open(fname, 'r')) as f:
        for i, l in enumerate(f):
            pass
    return i + 1


class Client():
    __metaclass__ = ABCMeta

    def __init__(self, subscriber) :
        self.subscriber = subscriber
        self.treshold = 0

        self.handlers = self.get_handlers()

    def subscribe(self, data='', keywords={}):
        self.subscriber.subscribe(data, self, keywords)

    def do_subscribe(self):
        if self.handlers is None:
            return

        #print self.handlers
        for handler in self.handlers:
            self.subscriber.add_handler(handler)
        #self.subscribe(keywords=self.keywords_passive)

    def get_handlers(self):
        return None
        #return {}

    def cancel(self):
        pass

class Standard(Client):

    def joke(self):
        n = randint(0, file_len(cst.JOKES_PATH)-1)

        with contextlib.closing(io.open(cst.JOKES_PATH, mode="r", encoding='utf-8')) as f: #use io for encoding in python 2
            for i, line in enumerate(f):
                if i == n:
                    return line

    def repeat(self):
         return cst.REPEAT.format(self.subscriber.last_answer)

    def what_heard(self):
        return cst.HEARD.format(self.subscriber.last_sentence)

    def time(self):
        now = datetime.datetime.now()
        return cst.TIME.format(now.hour, now.minute)

    def date(self):
        now = datetime.datetime.now()
        return cst.DATE.format(now.day, calendar.month_name[now.month])

    def reset(self):
        return cst.RESET

    def smile(self):
        self.subscriber.publish_action('1')
        return 'hehehehehehe'

class Quizz(Client):
    def __init__(self, subscriber):
        Client.__init__(self, subscriber)
        self.playing = False

        with contextlib.closing(io.open(cst.QUIZZ_PATH, mode="r", encoding='utf-8')) as f: #use io for encoding in python 2
            self.questions = f.read().splitlines()

        self.subscriber.subscribe_to_topic(KeywordsHandler(self.play_game, (AFFIRMATION,), extendable=True))
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.do_not_play_game, (NEGATION,), extendable=True))

        shuffle(self.questions)


    def cancel(self):
        if self.thread is not None:
            self.playing = True


    def get_handlers(self):

        self.thread = threading.Thread(target=self.play_quizz)
        self.thread.start()

        return None

    def play_quizz(self,):
        
        response = rospy.ServiceProxy('variable_info', VariableInfo)('curiosity')

        while float(response.value) > response.min:

            rospy.ServiceProxy('increment_variable', Increment)('curiosity', -10)
            rospy.ServiceProxy('increment_variable', Increment)('frustration', 5)
            response = rospy.ServiceProxy('variable_info', VariableInfo)('curiosity')

            time = cst.BASE_TIME_PLAY_GAME+((1-(float(response.value)/response.max))*cst.TIME_ADDITION_PLAY_GAME)
            print 'SLEEPING {}s'.format(time)
            print 'curiosity {}'.format(response.value)
            
            #print '{}+{}'.format(cst.BASE_TIME_PLAY_GAME, ((1-(float(response.value)/response.max))*cst.TIME_ADDITION_PLAY_GAME))

            self.subscriber.publish_data(choice(cst.ASK_PLAY_GAME))

            #self.subscriber.subscribe_to_topic(KeywordsHandler(self.true_callback, ((AFFIRMATION),), extendable=False))
            for i in xrange(int(time)):
                if self.playing:
                    return
                rospy.sleep(1)


        self.subscriber.publish_data(choice(cst.NO_PLAY_GAME))

    def do_not_play_game(self, data):
        rospy.ServiceProxy('increment_variable', Increment)('frustration', 15)

    def play_game(self, data):

        rospy.ServiceProxy('increment_variable', Increment)('curiosity', -2)
        self.playing = True

        self.subscriber.publish_data(choice(cst.HAPPY_PLAY_GAME))

        if not self.questions:
            self.subscriber.publish_data(cst.NO_QUESTIONS_PLAY_GAME)
            return

        question = self.questions[0].split('$')[0]
        self.correct_answer = self.questions[0].split('$')[1]

        del self.questions[0]

        self.subscriber.publish_data(question)

        tmp = []
        for word in self.correct_answer.split():
            tmp.append((word,))

        #self.subscriber.subscribe_to_topic(KeywordsHandler(self.true_callback, ((self.correct_answer,),), extendable=False))
        
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.true_callback, tuple(tmp), extendable=False))
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.wrong_callback, (('*',),), extendable=False))
         

    def true_callback(self, data):
        self.subscriber.publish_data(choice(cst.WIN_PLAY_GAME))
        self.ask_continue()

    def wrong_callback(self, data):
        self.subscriber.publish_data(choice(cst.LOSE_PLAY_GAME).format(self.correct_answer))
        self.ask_continue()

    def ask_continue(self):

        self.subscriber.publish_data(choice(cst.ASK_CONTINUE_PLAY_GAME))

        self.subscriber.subscribe_to_topic(KeywordsHandler(self.play_game, (AFFIRMATION,), extendable=True))
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.stop_play_game, (NEGATION,), extendable=True))

    def stop_play_game(self, data):
        self.subscriber.publish_data(choice(cst.STOP_PLAY_GAME))

class ListMaker(Client):

    class List():
        def __init__(self, name):
            self.actions = []

    def __init__(self, subscriber):
        Client.__init__(self, subscriber)
        self.lists = {}
        self.treshold = 2
        self.keywords = {'*' :1}

        #Possible names are located before the word list
        self.possible_names = ('to do', 'shopping')

        self.naming = ('name', 'call')

    def get_handlers(self):
        self.create_handler = KeywordsHandler(self.create_callback, (CREATION, ('list',)), extendable=True)
        self.add_handler = KeywordsHandler(self.create_callback, (ADD, ('list',)), extendable=True)

        return (self.create_handler, self.add_handler)
        #return {'create' :1, 'list' : 1}
    def get_name(self, data):
        for name in self.possible_names:
            print name + ' | ' + data
            if name in data:
                return name
                #return cst.CREATE_LIST_1.format(name)

        for naming in self.naming:
            for word in data.split():
                if naming in word:
                    return data.split(word)[1]
                    #return cst.CREATE_LIST_1.format(data.split(word)[1])

        data = data.split('list')
        print data
        if len(data) != 1:
            if data[1] != '':
                return data[1]
                #return cst.CREATE_LIST_2.format(data[1])

        return None

    def create_callback(self, data):


        self.current_name = self.get_name(data)


        if self.current_name is None:
            return self.what_name_callback()

        self.current_name = self.current_name.strip(' \t\n\r')

        if self.lists.get(self.current_name) is not None:
            self.subscriber.subscribe_to_topic(KeywordsHandler(self.select_existant_callback, (('select', 'ok', 'yes'),)))
            self.subscriber.subscribe_to_topic(KeywordsHandler(self.what_name_callback, (('another', 'no', 'not', 'other'),)))
            return cst.EXIST_LIST.format(self.current_name)


        return cst.CREATE_LIST_1.format(self.current_name)

    def create_callback_with_name(self, name):
        return cst.CREATE_LIST_1.format(name)

    def what_name_callback(self):
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.create_callback_with_name, (('*',),), extendable=True))
        self.asking = True
        return cst.ASK_NAME

    def select_existant_callback(self):
        return cst.CREATE_LIST_1.format(self.current_name)

    def add_callback(self):
        self.subscribe(keywords=self.keywords, data='what is the name of this list')

    #def callback(self, data):
        #data = data.split('list')
        #return 'your have created a list named "{}"'.format(data[1])


class Timer(Client):
    def __init__(self, subscriber):
        Client.__init__(self, subscriber)

        self.treshold = 2
        self.keywords = {'hour' :2, 'minute' : 2, 'second' :2}
        self.timer = None


    def get_handlers(self):
        self.partial_create_handler = KeywordsHandler(self.partial_create_callback, (CREATION, ('timer',)), extendable=True)
        self.create_handler = KeywordsHandler(self.create_callback, (CREATION, ('timer',), ('hour', 'minute', 'second')), extendable=True)

        return (self.create_handler, self.partial_create_handler)
        #return {'create' :1, 'set' :1, 'timer' : 1, 'hour' :0.5, 'minute' : 0.5, 'second' :0.5}

    #def first_call(self):
        #self.subscribe(keywords=self.keywords, data='How much time')

    def partial_create_callback(self, data):
        #self.subscribe(self.create_callback, ('hour', 'minute', 'second'))
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.create_callback, (('hour', 'minute', 'second'),), extendable=True))
        return cst.ASK_TIME

    def timer_callback(self):
        self.subscriber.publish_data(cst.TIMER_END)

    def cancel(self):
        if self.timer is not None:
            self.timer.cancel()

    def create_callback(self, data):

        data = text2int(data)
        #print data

        #data = data.split()

        duration = {key:0 for key in self.keywords}

        for index, word in enumerate(data):

            #Remove the plural form
            try:
                if word[-1] == 's':
                    word = word[:-1]
            except TypeError:
                pass

            if word in self.keywords:
                if index == 0:
                    return cst.ERROR_FORMAT_INDEX

                try:
                    duration[word] = int(data[index-1])
                    
                except ValueError:
                    return cst.ERROR_FORMAT

        time = (duration['hour']*60+duration['minute'])*60+duration['second']
        self.timer = threading.Timer(time, self.timer_callback)
        self.timer.start()

        #print '###### ' + str(duration) + ' ######'
   



        return cst.TIMER_SET.format(duration['hour'], duration['minute'], duration['second'])

class ChitChat(Client):
    def __init__(self, subscriber):
        Client.__init__(self, subscriber)

    def get_handlers(self):
        self.good_handler = KeywordsHandler(self.good, (PERSONNAL, EMOTIONS_GOOD))
        self.bad_handler = KeywordsHandler(self.bad, (PERSONNAL, EMOTIONS_BAD))
        self.angry_handler = KeywordsHandler(self.angry, (PERSONNAL, EMOTIONS_ANGRY))

        return (self.good_handler, self.bad_handler, self.angry_handler)

    def good(self, sentence):
        if is_negative(sentence):
            self.subscriber.publish_action("3")
            return choice(cst.FEELING_BAD)

        self.subscriber.publish_action("2")
        return choice(cst.FEELING_GOOD)

    def bad(self, sentence):
        if is_negative(sentence):
            self.subscriber.publish_action("2")
            return choice(cst.FEELING_GOOD)

        self.subscriber.publish_action("3")
        return choice(cst.FEELING_BAD)

    def angry(self, sentence):
        self.subscriber.publish_action("7")
        return choice(cst.FEELING_ANGRY)

    def ask(self):
        #Action already realised
        return choice(cst.ROBOT_FEELINGS)

class Conversation(Client):
    def __init__(self, subscriber):
        Client.__init__(self, subscriber)

        self.timer = None
        self.counter = 0


    def get_handlers(self):
        self.start_handler = KeywordsHandler(self.start, (START, TOPIC), extendable=True)
        self.end_handler = KeywordsHandler(self.end, (END, TOPIC), extendable=True)

        return (self.start_handler, self.end_handler)

    def start(self, data):
        self.timer = datetime.datetime.now()
        self.counter = self.subscriber.counter

        self.subscriber.publish_action("2")
        return choice(cst.START)

    def end(self, data):
        if self.timer is None:
            return cst.NO_DISCUSSION

        t = datetime.datetime.now() - self.timer

        hours, r = divmod(t.seconds, 3600)
        minutes, seconds = divmod(r, 60)

        self.timer = None

        self.subscriber.publish_action("6")
        return choice(cst.END1) + ' | ' + cst.END2.format(hours, minutes, seconds, self.subscriber.counter-self.counter)
    

if __name__ == '__main__':
    pass
