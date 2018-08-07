#!/usr/bin/env python
import requests
from synthesizer import Synthesizer
from os import environ
import rospy
#from threading import thread

class WatsonSynthesizer(Synthesizer):



	def __init__(self, path):
		Synthesizer.__init__(self, path)

		#self.timeout = 10

		self.user = environ.get('WATSON_USER_STT')
		self.passwd = environ.get('WATSON_PASS_STT')

		if self.user is None or self.passwd is None:
			raise ValueError("One of the Watson's credidentials is not set. Try 'export WATSON_USER_STT=\"xxx\"' and 'export WATSON_PASS_STT=\"xxx\"'")

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
