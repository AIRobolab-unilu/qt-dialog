#!/usr/bin/env python
import contextlib
import constants as cst

from random import randint, choice

import io
import threading
import datetime
import calendar
import json
import datetime

import time
from abc import ABCMeta, abstractmethod



CREATION = ('set', 'create', 'make')
ADD = ('add', 'append')

START = ('start', 'begin', 'have', "let's", 'let')
END = ('end', 'over', 'other') #the STT can get 'other' instead of 'over'

TOPIC = ('conversation', 'discussion', 'topic', 'talk')

class KeywordsHandler():
    def __init__(self, callback, keywords):
        self.callback = callback
        self.keywords = keywords

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
    with contextlib.closing(open(cst.JOKES_PATH, 'r')) as f:
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

        print self.handlers
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
        self.create_handler = KeywordsHandler(self.create_callback, (CREATION, ('list',)))
        self.add_handler = KeywordsHandler(self.create_callback, (ADD, ('list',)))

        return (self.create_handler, self.add_handler)
        #return {'create' :1, 'list' : 1}
    def get_name(self, data):
        for name in self.possible_names:
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
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.create_callback_with_name, (('*',),)))
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
        self.partial_create_handler = KeywordsHandler(self.partial_create_callback, (CREATION, ('timer',)))
        self.create_handler = KeywordsHandler(self.create_callback, (CREATION, ('timer',), ('hour', 'minute', 'second')))

        return (self.create_handler, self.partial_create_handler)
        #return {'create' :1, 'set' :1, 'timer' : 1, 'hour' :0.5, 'minute' : 0.5, 'second' :0.5}

    #def first_call(self):
        #self.subscribe(keywords=self.keywords, data='How much time')

    def partial_create_callback(self, data):
        #self.subscribe(self.create_callback, ('hour', 'minute', 'second'))
        self.subscriber.subscribe_to_topic(KeywordsHandler(self.create_callback, (('hour', 'minute', 'second'),)))
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


class Conversation(Client):
    def __init__(self, subscriber):
        Client.__init__(self, subscriber)

        self.timer = None


    def get_handlers(self):
        self.start_handler = KeywordsHandler(self.start, (START, TOPIC))
        self.end_handler = KeywordsHandler(self.end, (END, TOPIC))

        return (self.start_handler, self.end_handler)

    def start(self, data):
        self.timer = time.gmtime()

        return choice(cst.START)

    def end(self, data):
        if self.time is None:
            return cst.NO_DISCUSSION

        t = datetime.timedelta(seconds=time.gmtime() - self.timer)

        return choice(cst.END1) + ' | ' + cst.END2.format(t.hours, t.minutes, t.seconds)
    


if __name__ == '__main__':
    pass
