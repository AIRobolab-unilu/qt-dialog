#!/usr/bin/env python
import requests
from synthesizer import Synthesizer
from os import environ

class WatsonSynthesizer(Synthesizer):

	def __init__(self, path):
		Synthesizer.__init__(self, path)

		self.user = environ['WATSON_USER_STT']
		self.passwd = environ['WATSON_PASS_STT']

		self.url = 'https://stream.watsonplatform.net/speech-to-text/api/v1/recognize'
		
		self.headers = {'content-type': 'audio/wav', 'Accept-Charset': 'UTF-8'}
		

	def transcribe(self):

		r = requests.post(self.url, data=open(self.path), headers=self.headers, auth=(self.user, self.passwd))
		return r.json()

if __name__ == '__main__':
	pass
