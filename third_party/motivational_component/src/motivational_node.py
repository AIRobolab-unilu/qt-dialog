#!/usr/bin/env python

import rospy
import json
import contextlib
import time
from std_msgs.msg import *
from motivational_component.msg import * 
from motivational_component.srv import * 
import signal
import sys

try:
    from sensor_msgs.msg import BatteryState
except ImportError:
    SIMULATION = True
else:
    SIMULATION = False

import datetime as dt

class MotivationnalComponent() :

    def __init__(self):

        rospy.init_node('motivational', anonymous=True)

        print 'SIMULATION: {}'.format(SIMULATION)

        #Create timers at different rates

        self.config = self.load_json('../data/config.json')
        self.values = self.load_json('../data/assistive_mode.json')

        self.status = self.values['status']

        #self.values = self.load_json('../data/companion_mode.json')
        #self.values = self.load_json('../data/service_mode.json')

        self.control_loop_rate = 1
        self.robot_bumper         = 0
        self.day_cycle_fatigue = 0
        self.task_fatigue      = 0
        self.overall_fatigue   = 0
        self.curiosity_level   = 0
        self.frustration_time = 0

        self.robot_safe_status_level     = 10
        self.tired_level_status        = 0
        self.laptop_energy_fatigue     = 100
        self.robot_energy_fatigue       = 100

        self.robot_bumper = 1
        self.robot_pain_status_level = 0
        self.bumper_hit_simulation = 0

        self.fatigue_variables_normalization = 3


        self.pub = rospy.Publisher('motivation', Motivation, queue_size=10)

        #self.pub_laptop_fatigue = rospy.Publisher('computer_fatigue_level', Int32, queue_size=10)
        #self.pub_dvar_rest_level = rospy.Publisher('dvar_rest_level', Int32, queue_size=10)
        #self.pub_dvar_pain_status = rospy.Publisher('dvar_pain_level', Int32, queue_size=10)
        #self.pub_bvar_curiosity = rospy.Publisher('bvar_curiosity_level', Int32, queue_size=10)
        #self.pub_bvar_frustration = rospy.Publisher('bvar_frustration_level', Int32, queue_size=10)
        #self.pub_robot_fatigue_level = rospy.Publisher('robot_fatigue_level', Int32, queue_size=10)
        #self.pub_day_cycle_fatigue_level = rospy.Publisher('day_cycle_fatigue_level', Int32, queue_size=10)
        #self.pub_task_fatigue_level = rospy.Publisher('task_fatigue_level', Int32, queue_size=10)
        #self.pub_bumper_total_hits = rospy.Publisher('bumper_hits', Int32, queue_size=10)
        self.pub_message = rospy.Publisher('message_variable', MessageVariable, queue_size=10)


        rospy.Subscriber("/motivational_component/task_fatigue_activation", String, self.fatigue_task_activation_callback)
        rospy.Subscriber("/motivational_component/task_curiosity_activation", String, self.curiosity_task_activation_callback)
        rospy.Subscriber("/motivational_component/task_frustration_activation", String, self.frustration_task_activation_callback)

        rospy.Subscriber("laptop_charge", BatteryState, self.laptop_battery_status_callback)
        rospy.Subscriber("/motivational_component/task_frustration_update", String, self.frustration_update_callback)
        rospy.Subscriber("/bumper", Int32, self.updateBumperCounter_callback)
        signal.signal(signal.SIGINT, self.stop)
        self.running = True
        #rospy.Subscriber("/rate", String, self.rate_update_callback)


        self.run()

    def stop(self, signal, frame):
        self.running = False

    def increment_callback(self, query):
        self.status[query.name]['value'] += query.value
        return IncrementResponse()

    def variable_info_callback(self, query):
        variable = self.status[query.name]

        if variable.get('low_limit') is None:
            variable['low_limit'] = None

        if variable.get('high_limit') is None:
            variable['high_limit'] = None

        return VariableInfoResponse(variable['min'],
            variable['max'], variable['low_limit'], variable['high_limit'], str(variable['value']))

    def run(self):

        rospy.Service('increment_variable', Increment, self.increment_callback)
        rospy.Service('variable_info', VariableInfo, self.variable_info_callback)

        start_time = time.time()
        previous_time = int(start_time)
        while(self.running):
            elapsed_time = int(time.time() - start_time)
            if elapsed_time != previous_time:

                for key, variable in self.status.iteritems():
                    value = int(variable['value'])

                    #print key, variable
                    if value >= variable['max']:
                        self.pub_message.publish(MessageVariable('danger', 'high', key))
                    elif variable.get('high_limit') is not None:
                        if value >= variable['high_limit']:
                            self.pub_message.publish(MessageVariable('warning', 'high', key))
                    if value <= variable['min']:
                        self.pub_message.publish(MessageVariable('danger', 'low', key))
                    elif variable.get('low_limit') is not None:
                        if value <= variable['low_limit']:
                            self.pub_message.publish(MessageVariable('warning', 'low', key))
                    

                h = Header()
                h.stamp = rospy.Time.now()


                self.pub.publish(h, self.laptop_energy_fatigue, self.overall_fatigue,
                    self.robot_pain_status_level, self.curiosity_level, self.frustration_time,
                    self.robot_energy_fatigue, self.day_cycle_fatigue, self.task_fatigue, self.robot_bumper)

                #print self
                #print self.config
                for key, value in self.config.iteritems():
                    if value['active'] and elapsed_time%value['timer'] == 0:
                        #print '{} => {}s'.format(key, value['timer'])

                        getattr(self, key)()


            previous_time = elapsed_time

    def load_json(self, path):
        with contextlib.closing(open(path)) as json_data:
            return json.loads(json_data.read())

    # Motivational subsystem initialization
    def timer_motivational_callback(self):
        pass
    

    def fatigue_task_timer_callback(self):
        self.task_fatigue += 1
        self.day_cycle_fatigue += 1

        #self.pub_task_fatigue_level.publish(self.task_fatigue)
        #self.pub_day_cycle_fatigue_level.publish(self.day_cycle_fatigue)


    def curiosity_timer_callback(self): 
        #It starts at 0
        self.status['curiosity']['value'] += 1
        #self.curiosity_level += 1
        #self.pub_bvar_curiosity(self.curiosity_level)

        if self.status['curiosity']['value'] == 100:
            self.config['curiosity_timer_callback']['active'] = False

    
    def fatigue_human_rutine_timer_callback(self):
        #Human Routine
        #From 00:00 to 9:00 sleep: fatigue decrease:
        #From 9:00 to 15:00 fatigue increase: work
        #From 15:00 to 17:00 fatigue decrease: siesta
        #From 17:00 to 22:00 fatigue increase: work
        #From 22:00 to 00:00 fatigue increase x2 : work 
        hour = dt.datetime.now().hour

        if ((hour > self.values['constants']['morning_start'] 
            and hour < self.values['constants']['siesta_start'])
                or (hour > self.values['constants']['siesta_end'] 
                    and hour < self.values['constants']['dinner_start'])):

            self.day_cycle_fatigue += 1

        elif hour > self.values['constants']['dinner_start']:
            self.day_cycle_fatigue += 2
        elif (hour > self.values['constants']['launch_start'] 
            and hour < self.values['constants']['afternoon_start']):
            self.day_cycle_fatigue += 3
        else:
            self.day_cycle_fatigue += 2

        #self.pub_day_cycle_fatigue_level.publish(self.day_cycle_fatigue)

    
    def fatigue_overall_timer_callback(self):
        #value to normalize the results
        #each variable go from 0 to 100;
        self.status['fatigue']['value'] = ((self.laptop_energy_fatigue + self.robot_energy_fatigue + (100-self.day_cycle_fatigue))/self.fatigue_variables_normalization)
        #self.pub_dvar_rest_level.publish(self.overall_fatigue)

    
    def frustration_timer_callback(self):
        self.status['frustration']['value'] += 1
        #self.pub_bvar_frustration.publish(self.frustration_time)

        #Useless I guess
        #if self.frustration_time > self.values['frustration_time']['threshold']:
            #self.frustration_time = 80 # ??


    
    def pain_timer_callback(self):
        """! Pain Level subsystem initialization
            S
            0 : ok
            10: Pain
        """

        self.bumper_hit_simulation += 1
        if (SIMULATION and self.bumper_hit_simulation % 5 == 0):
            self.robot_bumper += 1

        if self.robot_bumper % self.values['constants']['max_bumper_hits'] == 0:
            self.robot_pain_status_level += 1
            #self.pub_dvar_pain_status.publish(self.robot_pain_status_level)

        #self.pub_bumper_total_hits.publish(self.robot_bumper)

    
    def robot_batt_simulation_timer_callback(self):
        self.robot_energy_fatigue -= 1
        #self.pub_robot_fatigue_level.publish(self.robot_energy_fatigue)

    
    def laptop_batt_simulation_timer_callback(self): 
        self.laptop_energy_fatigue -= 1

        #self.pub_laptop_fatigue.publish(self.laptop_energy_fatigue)


    #Callabck on subscribers
    def updateBumperCounter_callback(self, msg):
        self.robot_bumper += 1

        if(robot_bumper < 0):
            
            self.robot_safe_status_level += 1
            
            self.robot_bumper = 10;

    def laptop_battery_status_callback(self, msg):
        self.laptop_energy_fatigue = 100 - msg.percentage
        #self.pub_laptop_fatigue.publish(self.laptop_energy_fatigue)

    def robot_battery_status_callback(self, msg):
        self.robot_energy_fatigue = 100 - msg.percentage

        #self.pub_robot_fatigue_level.publish(self.robot_energy_fatigue)

    def frustration_task_activation_callback(self, msg):
        """
            Activate or deactivate the variable (0 or 1)
        """
        if msg.data == 1:
            self.status['frustration']['value'] = 0
            self.config['frustration_timer_callback']['active'] = True
        else:
            self.config['frustration_timer_callback']['active'] = False

    def frustration_update_callback(self, msg):
        """
            Activate or deactivate the variable (0 or 1)
        """
        self.frustration_level += msg.data

    def fatigue_task_activation_callback(self, msg):
        if msg.data == 1:
            self.task_fatigue = 0
            self.config['fatigue_task_timer_callback']['active'] = True
        else:
            self.config['fatigue_task_timer_callback']['active'] = False

    def curiosity_task_activation_callback(self, msg):
        """
            Activate or deactivate the variable (0 or 1)
        """
        if msg.data == 1:
            if self.curiosity_level > 0:
                self.curiosity_level = self.curiosity_level -10

            self.config['curiosity_timer_callback']['active'] = True
        else:
            self.config['curiosity_timer_callback']['active'] = False

    def __str__(self):
        return 'pain {}, curiosity: {}, frustration: {}, fatigue: {}'.format(
            self.status['pain'], self.status['curiosity'], self.status['frustration'], self.status['fatigue'],
            self.day_cycle_fatigue, self.overall_fatigue, self.task_fatigue)


    __repr__ = __str__

if __name__ == '__main__':
    MotivationnalComponent()
