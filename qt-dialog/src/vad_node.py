#!/usr/bin/python
import rospy
from std_msgs.msg import Bool
import os
import wave
import contextlib

import constants as cst

from vad.vad_webrtc_v1 import VADWebRTCV1
from vad.vad_webrtc_v2 import VADWebRTCV2

from naoqi_bridge_msgs.msg import AudioBuffer



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

        self.tools = {'pyaudio':self.classic_record, 'pocketsphinx':self.pocket_sphinx_record}
        
        

        self.listener()       

    def listener(self):
        rospy.Subscriber("speaking", Bool, self.callback)
        rospy.Subscriber("naoqi_microphone/audio_raw", AudioBuffer, self.audio_callback)
        rospy.spin()

    def record(self, *args, **kwargs):
        if self.get_audio is None:
            self.get_audio = self.tools.get(rospy.get_param("vad"))
            if self.get_audio is None:
                rospy.logerr(rospy.get_caller_id() + ' Wrong parameter "{}" for vad'.format(rospy.get_param("vad")))
                rospy.signal_shutdown(' Wrong parameter "{}" for vad'.format(rospy.get_param("vad")))
        #print args
        return self.get_audio(args)
        #return self.get_audio(args[0], args[1])

    def audio_callback(self, data):

        self.audio_buffer += data.data
        self.rate = data.frequency

        #print type(data)
        print data.data
        print len(data.data)
        print type(data.channelMap)
        print data.channelMap
        #print data.frequency

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}'.format(data.data))
        
        self.speaking = data.data
        self.tool.speaking = self.speaking

    def classic_record(self, *args, **kwargs):
        args = args[0]
       
        #print stream.read(chunk)
        return args[0].read(args[1]) #For PyAudio

    def pocket_sphinx_record(self):

        rospy.sleep(15)

        if len(self.audio_buffer) == 0:
            return

        n = int(self.rate*0.03)
        buff = []
        for i in xrange(len(self.audio_buffer)):
            #print i
            #if self.audio_buffer[0] < 0 :
                #self.audio_buffer[0] = self.audio_buffer[0] + 65536

            #buff = buff + chr(self.audio_buffer[0]%256)
            #buff = buff + chr( (self.audio_buffer[0] - (self.audio_buffer[0]%256)) /256)

            buff.append(str(self.audio_buffer[0]))

            del self.audio_buffer[0]

            if i >= 60000:
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


        



        return pcm_data

        


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
