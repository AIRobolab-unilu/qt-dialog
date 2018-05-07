#!/usr/bin/env python

import json
from comprehension_module import ComprehensionModule

import subprocess
import constants as cst
import contextlib
from os import environ

class WatsonNLP(ComprehensionModule):

    def __init__(self):
        ComprehensionModule.__init__(self)

        self.user = environ['WATSON_USER_NLP']
        self.passwd = environ['WATSON_PASS_NLP']


        
    def comprehend(self, text):

        #TODO: Construct Json from scratch with parameters insted of relying on a squelet
        #json_data=open(cst.WATSON_NLP_PARAM).read()

        #data = json.loads(json_data)

        with contextlib.closing(open(cst.WATSON_NLP_PARAM)) as json_data:
            data = json.loads(json_data.read())

        data['text'] = text

        with open(cst.WATSON_NLP_PARAM+'.tmp', 'w') as outfile:
            json.dump(data, outfile)

        process = subprocess.Popen(['bash', cst.WATSON_NLP_SCRIPT, self.user, self.passwd, cst.WATSON_NLP_PARAM+'.tmp', text], stdout=subprocess.PIPE)
        out, err = process.communicate()

        print out, err


        """response = self.natural_language_understanding.analyze(
          text='IBM is an American multinational technology company '
               'headquartered in Armonk, New York, United States, '
               'with operations in over 170 countries.',
          features=Features(
            entities=EntitiesOptions(
              emotion=True,
              sentiment=True,
              limit=2),
            keywords=KeywordsOptions(
              emotion=True,
              sentiment=True,
              limit=2)))

        print(json.dumps(response, indent=2))
                #return r.json()"""

if __name__ == '__main__':
    print WatsonNLP().comprehend('I love potatoes') 
