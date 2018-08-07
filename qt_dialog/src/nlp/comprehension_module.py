#!/usr/bin/env python

from abc import ABCMeta, abstractmethod

class ComprehensionModule():
	__metaclass__ = ABCMeta

	def __init__(self):
		pass

	@abstractmethod
	def comprehend(self, data) :
		pass
