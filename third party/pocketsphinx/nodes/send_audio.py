#!/usr/bin/python

from time import sleep


import pyaudio
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



        # All set. Publish to topic
        self.transfer_audio_msg()

    def speaking_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + ' I heard {}'.format(data.data))
        self.speaking = data.data
        self.buf = None
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
                self.pub_.publish(self.buf)
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
