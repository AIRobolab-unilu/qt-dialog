#!/usr/bin/env python
import rospy

try:
    from svox_tts.srv import Speech
except ImportError:
	rospy.sleep(5)
	print 'Could not import {} from {}'.format('Speech', 'svox_tts.srv')

import rosservice


class Pico():

    def __init__(self):
    	service_name = '/svox_tts/speech'
    	while service_name not in rosservice.get_service_list():
    		print 'waiting for '+service_name
    		rospy.sleep(5)
        self.service = rospy.ServiceProxy(service_name, Speech)

        self.service(2, 'test', 100)
        self.service(3, 'test', 140)

    def say(self, arg):
        self.service(5, arg, 0)
        

if __name__ == '__main__':
    pass
