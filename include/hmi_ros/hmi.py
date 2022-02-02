#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from .outputs import *
from .inputs import *
from hmi_ros.msg import outCmd

Fs = 50 # Hz
node_name = 'hmi_node'
enabled = False
inputs = []
outputs = []

def NewOutputOrder(data):
    rospy.loginfo(data.outputs)
    rospy.loginfo(data.type)
    rospy.loginfo(data.params)

def hmi_start():
    global out_blue, out_green, in_green, in_green, out_coil

    rospy.init_node(node_name)
    GPIO.setmode(GPIO.BOARD)
    ConfigureHMI()
    rospy.Subscriber(node_name + '/output_order', outCmd, NewOutputOrder)
    rate = rospy.Rate(Fs) # 10hz


    # out_blue.Config(time_on = 100, time_on_unit = '%', period = 0.8, seq_length = 4, length_unit = 's', respawn = 0)

    while enabled and not rospy.is_shutdown():
        for o in outputs:
            o.Tick()
        rate.sleep()

    # Cleanup and exit
    for o in outputs:
        o.SetOutput(o.default_state)

    GPIO.cleanup()
    
def ConfigureHMI():
    global enabled, inputs, outputs, Fs

    if(rospy.has_param(node_name)):
        enabled = True
        Fs = rospy.get_param('sample_freq', 50)     # Default value is 50 Hz
        inputsConfig = (rospy.get_param('hmi_node/inputs'))  # Get list of dictionaries with the inputs from parameter server
        outputsConfig = (rospy.get_param('hmi_node/outputs')) # Get list of dictionaries with the outputs from parameter server
        # Try to add each input and output
        for i in inputsConfig.items():
            AddInput(i)     
        for i in outputsConfig.items():
            AddOutput(i)
    else:
        enabled = False
        rospy.logerr('No configuration parameter found. Parameter server should have parameter with same name as hmi node and include the inputs and outputs to configure.')


def AddInput(config):
    global inputs
    name = config[0]
    topic = config[1]['topic_notify']
    pin = config[1]['pin']
    bncTime = config[1]['bounce_time']
    actLow = config[1]['active_low']
    
    if not(isinstance(topic,str) and isinstance(pin, int) and isinstance(bncTime, int) and isinstance(actLow, bool)):
        rospy.loginfo('Unable to configure input %s. Bad parameter format.', name)
    else:
        inputs.append(Input(name, topic, pin, bncTime, actLow))
        rospy.loginfo('Added input')
    
def AddOutput(config):
    global outputs
    name = config[0]
    pin = config[1]['pin']
    defSt = config[1]['default_state']

    if not(isinstance(pin, int) and isinstance(defSt, int)):
        rospy.loginfo('Unable to configure output %s. Bad parameter format.', name)
    else:
        inputs.append(Toggling_Output(name, pin, Fs, defSt))
        rospy.loginfo('Added output')


