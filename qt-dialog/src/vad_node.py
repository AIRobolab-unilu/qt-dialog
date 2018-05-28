#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
import os
import wave
import contextlib


import constants as cst


from vad.vad_webrtc_v1 import VADWebRTCV1
from vad.vad_webrtc_v2 import VADWebRTCV2
import numpy as np
import sys
import struct

from threading import Lock

from random import random


try:
    import pyaudio                   
except ImportError:
    print 'Could not import {}'.format('pyaudio')





class VAD():

    def __init__(self):

        self.get_audio = None

        self.pub = rospy.Publisher('detection', Bool, queue_size=10)
        self.rate = 0

        rospy.init_node('vad', anonymous=True)

        #Create a thred because we need to listen to the "speaking" topic
        self.tool = VADWebRTCV2(self) #Set itself as the observer
        self.tool.start() #Start the thread

        self.speaking = False
        self.audio_buffer = []
        self.lock = Lock()

        print 'TOOLS ##########################'
        self.tools = {'pyaudio':self.classic_record}
        print self
        print 'DONE #################'
        self.topic_recording = {'micro': False, 'topic': True}

        self.audio_messages = {}
        try:
            from naoqi_bridge_msgs.msg import AudioBuffer
            self.audio_messages['AudioBuffer'] = AudioBuffer              
        except ImportError:
            print 'Could not import {} from {}'.format('AudioBuffer', 'naoqi_bridge_msgs.msg')

        try:
            from audio_common_msgs.msg import AudioData
            self.audio_messages['AudioData'] = AudioData                   
        except ImportError:
            print 'Could not import {} from {}'.format('AudioData', 'audio_common_msgs.msg')
        
        self.p = pyaudio.PyAudio()
        self.stream = None
        self.chunk_size = int(16000 * 30 / 1000)*2
        self.b = '' 

        self.pepper_channels = {'CHANNEL_FRONT_LEFT': '00',
                                'CHANNEL_FRONT_CENTER': '01',
                                'CHANNEL_FRONT_RIGHT': '02',
                                'CHANNEL_REAR_LEFT': '03',
                                'CHANNEL_REAR_CENTER': '04',
                                'CHANNEL_REAR_RIGHT': '05',
                                'CHANNEL_SURROUND_LEFT': '06',
                                'CHANNEL_SURROUND_RIGHT': '07',
                                'CHANNEL_SUBWOOFER': '08',
                                'CHANNEL_LFE': '09'}

        self.channel = 'CHANNEL_REAR_LEFT'

        self.listener()       

    def listener(self):
        rospy.Subscriber("speaking", Bool, self.callback)

        if self.topic_recording[rospy.get_param("input")]:
            rospy.loginfo(rospy.get_caller_id() + ' Subscribing to "{}" with the message {}'.format(rospy.get_param("input_topic"), rospy.get_param('input_topic_msg')))

            if rospy.get_param('input_topic_msg') == 'AudioBuffer':
                rospy.Subscriber(rospy.get_param("input_topic"), AudioBuffer, self.callback_audio_buffer)
            elif rospy.get_param('input_topic_msg') == 'AudioData':
                rospy.Subscriber(rospy.get_param("input_topic"), AudioData, self.callback_audio_data)
            else:
                rospy.logerr(rospy.get_caller_id() + ' Wrong parameter "{}" for input_topic_msg'.format(rospy.get_param("input_topic_msg")))
            

        #rospy.Subscriber("/naoqi_driver/audio", AudioBuffer, self.audio_callback)
        rospy.spin()

    def record(self, *args, **kwargs):
        if self.get_audio is None:
            print self
            self.get_audio = self.tools.get(rospy.get_param("vad"))
            if self.get_audio is None:
                rospy.logerr(rospy.get_caller_id() + ' Wrong parameter "{}" for vad'.format(rospy.get_param("vad")))
                rospy.signal_shutdown(' Wrong parameter "{}" for vad'.format(rospy.get_param("vad")))
                sys.exit(1)

        #print args
        return self.get_audio(args)
        #return self.get_audio(args[0], args[1])


    def select_channel(self, channels):
        return channels[0]

    def callback_audio_buffer(self, data):

        if self.speaking:
            self.b = ''
        

        self.audio_buffer += data.data
        self.rate = data.frequency

        #channel_map = [elem.encode("hex") for elem in data.channelMap]
        #print channel_map

        #if self.stream is None:

            
            #self.stream = self.p.open(format=pyaudio.paInt16,
                        #channels=1,
                        #rate=self.rate,
                        #output=True)

        #size = len(data.data)/len(data.channelMap)
        #size = len(data.data)
        #print size
        #tmp = [data.data[i] for i in xrange(size) if random() > (data.frequency-32000+0.0)/data.frequency]

        num_channel = len(data.channelMap)

        channels = [[] for i in xrange(num_channel)]
        [channels[i%num_channel].append(data.data[i]) for i in xrange(len(data.data))] #To test a particular channel

        #tmp = [0 for i in xrange(len(data.data)/num_channel)]
        #tmp = [tmp[i] + data.data[i+j]/num_channel for j in xrange(num_channel) for i in xrange(len(data.data)/num_channel)] 

        #for i in xrange(len(data.data)/num_channel):
            #for j in xrange(num_channel):
                #tmp[i] += data.data[i+j]

        #for c in tmp:
            #tmp[i] = tmp[i]/num_channel

        #tmp = [sum(channels[:][i]) for i in xrange(len(data.data)/num_channel)]

        #print tmp
        #for i in xrange(len(data.data)):
            #channels.append(data.data[size*i:size*(i+1)])

            #print size*i
            #print size*(i+1)
            #print len(data.data[size*i:size*(i+1)])


        #num = self.pepper_channels.get(self.channel)

        #if num is None:
            #rospy.logerr(rospy.get_caller_id() + ' Wrong channel "{}" for pepper_channels'.format(self.channel))
            #rospy.signal_shutdown(' Wrong channel "{}" for pepper_channels'.format(self.channel))
            #sys.exit(1)

        #if num not in channel_map:
            #rospy.logerr(rospy.get_caller_id() + ' The channel "{}" is not used by to stream audio'.format(self.channel))
            #rospy.signal_shutdown(' The channel "{}" is not used by to stream audio'.format(self.channel))
            #sys.exit(1)

        #tmp = channels[channel_map.index(num)]
        #tmp = channels[1]

        #tmp = [data.data[i] for i in xrange(size) if not (i % int((data.frequency-32000+0.0)/data.frequency*10)) == 0] #From 48K HZ to 32K HZ

        #print int(10-(data.frequency-32000+0.0)/data.frequency*10)
        #print 'size '+str(len(tmp))
        #tmp = data.data
        #tmp = [data.data[i] for i in xrange(size*3, size*4)]
        #print sum(data.data[:size])
        #print sum(data.data[size:size*2])
        #print sum(data.data[:size*2:size*3])
        #print sum(data.data[size*3:size*4])

        #print sum(data.data)
        #tmp = data.data

        #buff = struct.pack('f'*len(tmp), *tmp)

        tmp = self.select_channel(channels)

        buff = struct.pack('<' + ('h' * len(tmp)), *tmp)
        if self.b == []:
            self.b = buff
        else:
            self.b += buff
        #buff = ''.join([str(s) for s in np.int16(data.data)])
        #print type(data.data)
        #print type(data.data[0])
        #print buffx
        #self.stream.write(buff)

        #print len(self.b)
        #self.stream.write(buff)

        if len(self.b) > 200000:

            with contextlib.closing(wave.open('test.wav', 'wb')) as wf:
                #print 'saving ...'
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(self.b)
        #exit(0)

        #print type(data)
        #print data.data
        #print len(data.data)
        #print type(data.channelMap)
        #print len(data.channelMap)
        #a = [elem.encode("hex") for elem in data.channelMap]
        #print a
        #print data.channelMap
        #print data.frequency

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}'.format(data.data))
        
        self.speaking = data.data
        self.tool.speaking = self.speaking

    def classic_record(self, *args, **kwargs):
        args = args[0]

        if not self.topic_recording[rospy.get_param("input")]:
            return args[0].read(args[1])

        return self.get_topic_buffer()

        #self.buffer_record()
       
        #print type(args[0].read(args[1]))
        #print args[0].read(args[1])

    def get_topic_buffer(self):
        #print 'init done'

        while len(self.b) < self.chunk_size:
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

        

        #print len(data)

        dt = np.dtype(np.int16)
        #print np.frombuffer(data, dtype=dt)

        
        return data #For PyAudio

    def buffer_record(self):
        rospy.sleep(5)

        

        if len(self.audio_buffer) == 0:
            return

        n = int(self.rate*0.03)
        #buff = []
        buff = ''
        for i in xrange(len(self.audio_buffer)/4):
            #print i
            #if self.audio_buffer[0] < 0 :
                #self.audio_buffer[0] = self.audio_buffer[0] + 65536

            #buff += chr(self.audio_buffer[0]%256)
            #buff += chr( (self.audio_buffer[0] - (self.audio_buffer[0]%256)) /256)

            #buff.append(str(self.audio_buffer[0]))

            

            del self.audio_buffer[0]

            if i >= 30000:
                break
        buff = ' '.join(buff)

        with contextlib.closing(wave.open('test.wav', 'wb')) as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(16000)
            wf.writeframes(buff)

        print len(self.audio_buffer)

        with contextlib.closing(wave.open('test.wav', 'rb')) as wf:

            assert wf.getnchannels() == 1, (
                '{0}: sound format is incorrect! Sound must be mono.'.format(
                    file_name))

            assert wf.getsampwidth() == 2, (
                '{0}: sound format is incorrect! '
                'Sample width of sound must be 2 bytes.').format(file_name)

            assert wf.getframerate() in (8000, 16000, 32000), (
                '{0}: sound format is incorrect! '
                'Sampling frequency must be 8000 Hz, 16000 Hz or 32000 Hz.')

            num_channels = wf.getnchannels()
            assert num_channels == 1
            sample_width = wf.getsampwidth()
            assert sample_width == 2
            sample_rate = wf.getframerate()
            assert sample_rate in (8000, 16000, 32000)
            pcm_data = wf.readframes(wf.getnframes())


            #self.stream.stop_stream()
            #self.stream.close()

            #self.p.terminate()
            print 'DONE'   


    def notify(self, data, sample_rate) :
        """Writes a .wav file.
        Takes path, PCM audio data, and sample rate.
        """

        if self.speaking:
            return

        with contextlib.closing(wave.open(cst.OUTPUT_FILE, 'wb')) as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(sample_rate)
            wf.writeframes(data)
            self.pub.publish(True)

if __name__ == '__main__':
    try:
        VAD()
    except rospy.ROSInterruptException:
        pass
