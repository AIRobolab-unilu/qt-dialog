#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import constants as cst
from nlp.watson_nlp import WatsonNLP

class NLP():

    def __init__(self, tool):
        self.tool = tool

        rospy.init_node('nlp', anonymous=True)
        self.pub = rospy.Publisher('comprehension', String, queue_size=10)
        self.listener()

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + 'I heard {}, trying to comprehend the sentence ...'.format(data.data))
        
        self.tool.comprehend(data.data)

    def listener(self):
        rospy.Subscriber("transcription", String, self.callback)
        rospy.spin()

if __name__ == '__main__':
    NLP(WatsonNLP())

