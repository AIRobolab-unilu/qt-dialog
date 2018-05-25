#!/usr/bin/python

from time import sleep
import numpy as np
import sys
import struct

try:
    import pyaudio                   
except ImportError:
    print 'Could not import {}'.format('pyaudio')

try:
    from naoqi_bridge_msgs.msg import AudioBuffer                   
except ImportError:
    print 'Could not import {} from {}'.format('AudioBuffer', 'naoqi_bridge_msgs.msg')

try:
    from audio_common_msgs.msg import AudioData                    
except ImportError:
    print 'Could not import {} from {}'.format('AudioData', 'audio_common_msgs.msg')

from std_msgs.msg import String, Bool

import rospy
import sys



class AudioMessage(object):
    """Class to publish audio to topic"""

    def __init__(self):

        # Initializing publisher with buffer size of 10 messages
        self.pub_ = rospy.Publisher("sphinx_audio", String, queue_size=10)

        # initialize node
        rospy.init_node("audio_control")
        # Call custom function on node shutdown
        rospy.on_shutdown(self.shutdown)

        rospy.Subscriber("speaking", Bool, self.speaking_callback)
        
        self.speaking = False
        self.b = ''
        self.topic_recording = {'micro': False, 'topic': True}
        self.chunk_size = int(16000 * 30 / 1000)*2

        self.audio_messages = {'AudioBuffer': AudioBuffer, 'AudioData': AudioData}

       
        if self.topic_recording[rospy.get_param("input")]:
            rospy.loginfo(rospy.get_caller_id() + ' Subscribing to "{}" with the message {}'.format(rospy.get_param("input_topic"), rospy.get_param('input_topic_msg')))

            if rospy.get_param('input_topic_msg') == 'AudioBuffer':
                rospy.Subscriber(rospy.get_param("input_topic"), AudioBuffer, self.callback_audio_buffer)
            elif rospy.get_param('input_topic_msg') == 'AudioData':
                rospy.Subscriber(rospy.get_param("input_topic"), AudioData, self.callback_audio_data)
            else:
                rospy.logerr(rospy.get_caller_id() + ' Wrong parameter "{}" for input_topic_msg'.format(rospy.get_param("input_topic_msg")))



        # All set. Publish to topic
        self.transfer_audio_msg()


    def callback_audio_buffer(self, data):

        if self.speaking:
            self.b = ''
        

        
        self.rate = data.frequency

        num_channel = len(data.channelMap)

        channels = [[] for i in xrange(num_channel)]
        [channels[i%num_channel].append(data.data[i]) for i in xrange(len(data.data))] #To test a particular channel


        tmp = channels[0]

        buff = struct.pack('<' + ('h' * len(tmp)), *tmp)
        if self.b == ''
            self.b = buff
        else:
            self.b += buff



    def speaking_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' I heard {}'.format(data.data))
        self.speaking = data.data
        self.buf = ''
        #if self.speaking:
            #self.close_stream()
        #else:
            #self.open_stream()

    def open_stream(self):
        self.stream = self.audio.open(format=pyaudio.paInt16, channels=1,
                                    rate=16000, input=True)
        self.stream.start_stream()

    def close_stream(self):
        self.stream.stop_stream()
        self.stream.close()

    def get_topic_buffer(self):
        while len(self.b) < self.chunk_size:
            #print len(self.b)
            pass

        #print 'sending buffer'

        tmp = self.b[:self.chunk_size]
        self.b = self.b[self.chunk_size:]

        #print len(self.b)
        #print len(tmp)
        #print len(self.b)

        #data = args[0].read(args[1])

        #print type(data)
        #print type(self.b)
        #self.stream.write(tmp)
        return tmp

    def transfer_audio_msg(self):
        """Function to publish input audio to topic"""

        rospy.loginfo("audio input node will start after delay of 5 seconds")
        sleep(2)

        # Params
        self._input = "~input"
        _rate_bool = False

        # Checking if audio file given or system microphone is needed
        if rospy.has_param(self._input):
            if rospy.get_param(self._input) != ":default":
                _rate_bool = True
                self.stream = open(rospy.get_param(self._input), 'rb')
                rate = rospy.Rate(5) # 10hz
            else:
                # Initializing pyaudio for input from system microhpone
                self.audio = pyaudio.PyAudio()
                self.open_stream()
                
        else:
            rospy.logerr("No input means provided. Please use the launch file instead")


        while not rospy.is_shutdown():

            if self.speaking:
                #sys.stdout.write('X')
                self.buf = None
                continue

            try:

                self.buf = self.stream.read(1024)
            except IOError:
                pass

            if self.speaking:
                #sys.stdout.write('X')
                self.buf = None
                rospy.sleep(10)
                continue
            sys.stdout.write('_')

            if self.buf:
                # Publish audio to topic
                sys.stdout.write('_')

                tmp = self.get_topic_buffer()
                #print tmp
                self.pub_.publish(tmp)
                #self.pub_.publish(self.buf)
                if _rate_bool:
                    rate.sleep()
            else:
                rospy.loginfo("Buffer returned null")
                break

    @staticmethod
    def shutdown():
        """This function is executed on node shutdown."""
        # command executed after Ctrl+C is pressed
        rospy.loginfo("Stop ASRControl")
        rospy.sleep(1)


if __name__ == "__main__":
    AudioMessage()
