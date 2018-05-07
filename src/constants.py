#!/usr/bin/python

from os import path

DATA_DIR = '../data'
SCRIPTS_DIR = '../scripts'

##########################################################
#Path to files

OUTPUT_FILE = path.abspath(path.join(DATA_DIR, "output.wav"))

WATSON_NLP_PARAM = path.abspath(path.join(DATA_DIR, "parameters.json"))
WATSON_NLP_SCRIPT = path.abspath(path.join(SCRIPTS_DIR, "watson_request.bash"))

INTENTS_JSON = path.abspath(path.join(DATA_DIR, "intents.json"))
ACTIONS_JSON = path.abspath(path.join(DATA_DIR, "actions.json"))
SENTENCES_JSON = path.abspath(path.join(DATA_DIR, "sentences.json"))

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
END2 = 'our discussion lasted {} hour {} minutes and {} seconds'