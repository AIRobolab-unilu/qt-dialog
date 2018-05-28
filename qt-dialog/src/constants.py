#!/usr/bin/env python

from os import path

DATA_DIR = '../data'
SCRIPTS_DIR = '../scripts'
GRAMMAR_DIR = DATA_DIR+'/grammar'

##########################################################
#Path to files

OUTPUT_FILE = path.abspath(path.join(DATA_DIR, "output.wav"))

WATSON_NLP_PARAM = path.abspath(path.join(DATA_DIR, "parameters.json"))
WATSON_NLP_SCRIPT = path.abspath(path.join(SCRIPTS_DIR, "watson_request.bash"))

INTENTS_JSON = path.abspath(path.join(DATA_DIR, "intents.json"))
ACTIONS_JSON = path.abspath(path.join(DATA_DIR, "actions.json"))
SENTENCES_JSON = path.abspath(path.join(DATA_DIR, "sentences.json"))

GRAMMAR_FILE = path.abspath(path.join(GRAMMAR_DIR, "tmp.gram"))

JOKES_PATH = path.abspath(path.join(DATA_DIR, "jokes.txt"))

##########################################################
#Answers

#Standard
REPEAT = 'i said {}'
HEARD = 'you said {}'
TIME = 'it is {} hour and {} minutes'
DATE = 'we are the {} of {}'
RESET = 'our conversation has been reset'

#List Maker
CREATE_LIST_1 = 'the {} list has been created'
CREATE_LIST_2 = 'the list {} has been created'

EXIST_LIST = 'the list {} already exists | do you want to select this list or create a new one'

ASK_NAME = 'what is the name of this list'

#Timer
CREATE_TIMER = 'the {} list has been created'

ASK_TIME = 'how much time'

TIMER_END = 'timer is finsished'
for i in xrange(20):
    TIMER_END += ' ding'

ERROR_FORMAT = 'Incorrect format'
ERROR_FORMAT_INDEX = 'Incorrect format index'

TIMER_SET = 'the timer is set for {} hour {} minutes and {} seconds'

#Chit Chat
FEELING_GOOD = [
'i am also feeling good',
'so we are both feeling well'
]

FEELING_BAD = [
'i want to cry when i see you like that',
'maybe if i tell you that we are friends it will pump you up'
]

FEELING_ANGRY = [
'no need to be angry | be cool'
]

ROBOT_FEELINGS = [
"I am fine thank you | And you",
"I am okay just my arm glitching as usual | What about you"
]

ROBOT_DEMO = [
"It's going great | I am either impressing people I am talking to or boring them | Either way they are reacting",
"I am not quite sure I may be scaring some of the people I am talking to "
]


#Discussion
NO_DISCUSSION = 'our discussion has not started yet'

START = [
'yes i was waiting for you',
'cool i wanted to talk',
'nice to see you'
]

END1 = [
'bye bye',
'see you soon',
'it was fun talking to you'
]
END2 = 'our discussion lasted {} hour {} minutes and {} seconds and i understood {} sentences'


######################
#GRAMMAR HEADER
GRAMMAR_HEADER = "#JSGF V1.0;\n\n/**\n * JSGF Grammar Auto Generated - Do not touch\n */\n\ngrammar asr;\npublic <rule> =  "