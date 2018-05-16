#!/usr/bin/python
import rospy
from std_msgs.msg import Bool

import pyaudio
import collections
import contextlib
import sys
import wave
import webrtcvad
import threading


from abc import ABCMeta, abstractmethod

class VADListener(threading.Thread) :
    __metaclass__ = ABCMeta

    def __init__(self, observer) :
        threading.Thread.__init__(self)
        self.observer = observer
        self.speaking = False

    #def write_wave(self, path, audio, sample_rate):
        """
        Writes a .wav file.
        Takes path, PCM audio data, and sample rate.
        """
        
       
        #with contextlib.closing(wave.open(path, 'wb')) as wf:
            #wf.setnchannels(1)
            #wf.setsampwidth(2)
            #wf.setframerate(sample_rate)
            #wf.writeframes(audio)

    def read_wave(self, path):
        """Reads a .wav file.
        Takes the path, and returns (PCM audio data, sample rate).
        """
        with contextlib.closing(wave.open(path, 'rb')) as wf:
            num_channels = wf.getnchannels()
            assert num_channels == 1
            sample_width = wf.getsampwidth()
            assert sample_width == 2
            sample_rate = wf.getframerate()
            assert sample_rate in (8000, 16000, 32000)
            pcm_data = wf.readframes(wf.getnframes())
        return pcm_data, sample_rate

    def notify_observer(self, audio, sample_rate) :
        self.observer.notify(audio, sample_rate)


if __name__ == '__main__':
    pass
