#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import wave
import pyaudio
import sys
import webrtcvad

from abc import ABCMeta, abstractmethod
from vad_listener import VADListener

class WebRTC(VADListener) :
    __metaclass__ = ABCMeta


    def __init__(self, observer, tmp_filename = "tmp.wav") :
        VADListener.__init__(self, observer)

        self.audio = pyaudio.PyAudio()

        self.tmp_filename = tmp_filename

        self.stream = self.audio.open(format=self.format,
                        channels=self.channels,
                        rate=self.rate,
                        start = self.starting,
                        input=True,
                        frames_per_buffer=self.chunk)

        self.running = True

    def run(self):
        while self.running:
            if self.speaking:
                continue

            self.process_chunk()

        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

    @abstractmethod
    def process_chunk(self) :
        pass
    


if __name__ == '__main__':
    pass
