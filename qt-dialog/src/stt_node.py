#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool, String
import os

from stt.watson_synthesizer import WatsonSynthesizer
from stt.amazon_synthesizer import AmazonSynthesizer
from stt.google_synthetizer import GoogleSynthesizer
from stt.pocket_sphinx_synthesizer import PocketSphinxSynthesizer


import constants as cst

class STT():

    def __init__(self, tool):
        self.tool = tool

        rospy.init_node('stt', anonymous=True)
        self.pub = rospy.Publisher('transcription', String, queue_size=10)
        self.listener()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}, trying to transcript the audio ...'.format(data.data))

        json = self.tool.transcribe()

        if json['results'] :
            #print json['results']
            result = json['results'][0]['alternatives'][0]
            print 'Results : '
            print 'Confidence : ', str(result['confidence']*100)+'%'
            print 'Transcription : ', result['transcript']
            self.pub.publish(result['transcript'])
        else :
            print json
            print "No clue about what you're saying ..."

    def listener(self):      
        rospy.Subscriber("detection", Bool, self.callback)
        rospy.spin()

if __name__ == '__main__':
    STT(GoogleSynthesizer(cst.OUTPUT_FILE))
    #STT(WatsonSynthesizer(cst.OUTPUT_FILE))
