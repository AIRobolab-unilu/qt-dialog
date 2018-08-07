#!/usr/bin/env python
from std_msgs.msg import String

from abc import ABCMeta

class Synthesizer():
    __metaclass__ = ABCMeta

    def __init__(self, path):
        
        self.path = path

if __name__ == '__main__':
    pass
