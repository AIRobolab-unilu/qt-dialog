#!/usr/bin/env python
import requests
from synthesizer import Synthesizer
from os import environ
import rospy

import io

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
#from threading import thread

class GoogleSynthesizer(Synthesizer):
    def __init__(self, path):
        Synthesizer.__init__(self, path)

        #self.timeout = 10

        #self.user = environ.get('GOOGLE_USER_STT') #Do smth with it
        #self.passwd = environ.get('GOOGLE_PASS_STT') #Do smth with it

        self.client = speech.SpeechClient()


        #if self.user is None or self.passwd is None:
            #raise ValueError("One of the Google's credidentials is not set. Try 'export GOOGLE_USER_STT=\"xxx\"' and 'export GOOGLE_PASS_STT=\"xxx\"'")

    def timeout(self):
        #Start a thread to lookup and cancel the request if it exceeds the timeout
        pass
        
    def transcribe(self):

        with io.open(self.path, 'rb') as audio_file:
            content = audio_file.read()

        audio = types.RecognitionAudio(content=content)
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code='en-US')

        print 'sending data ...'
        response = self.client.recognize(config, audio)
        print 'received data ...'

        for result in response.results:
            print 'Transcript: {}'.format(result.alternatives[0].transcript)
            return result.alternatives[0].transcript

if __name__ == '__main__':
    GoogleSynthesizer('test')
