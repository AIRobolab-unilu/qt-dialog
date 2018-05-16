#!/usr/bin/env python
import requests
from synthesizer import Synthesizer
from os import environ
#from threading import thread

class WatsonSynthesizer(Synthesizer):



	def __init__(self, path):
		Synthesizer.__init__(self, path)

		#self.timeout = 10

		self.user = environ['WATSON_USER_STT']
		self.passwd = environ['WATSON_PASS_STT']

		self.url = 'https://stream.watsonplatform.net/speech-to-text/api/v1/recognize'
		
		self.headers = {'content-type': 'audio/wav', 'Accept-Charset': 'UTF-8'}


	def timeout(self):
		#Start a thread to lookup and cancel the request if it exceeds the timeout
		pass
		
	def transcribe(self):

		#self.now = datetime.datetime.now()

		#t = Thread(self.timeout)

		r = requests.post(self.url, data=open(self.path), headers=self.headers, auth=(self.user, self.passwd), timeout=15)
		return r.json()

if __name__ == '__main__':
	pass
