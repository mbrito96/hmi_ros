#!/usr/bin/env python

import rospy
import re
import RPi.GPIO as GPIO
from .outputs import *
from .inputs import *
from hmi_ros.msg import outCmd

Fs = 50 # Hz
node_name = 'hmi_node'
enabled = False
inputs = []
outputs = []
# pub_blue = None
# msg_blue = outCmd()

def hmi_start():
    global inputs, outputs, pub_blue

    rospy.init_node(node_name)
    GPIO.setmode(GPIO.BOARD)
    ConfigureHMI()
    rospy.Subscriber(node_name + '/output_order', outCmd, NewOutputOrder)
    rate = rospy.Rate(Fs) # 10hz

    # rospy.Subscriber('hmi_node/inputs/blue_button', String, ButtonPressed)
    # pub_blue = rospy.Publisher('/hmi_node/output_order', outCmd, queue_size=10)

    while enabled and not rospy.is_shutdown():
        for o in outputs:
            o.Tick()
        rate.sleep()

    # Cleanup and exit
    for o in outputs:
        o.SetOutput(o.default_state)

    GPIO.cleanup()
    
# def ButtonPressed(data):
#     s = data.data
#     if(s == 'SP'):
#         msg_blue.outputs = ['green_led', 'blue_led']
#         msg_blue.type = 'TOGGLE'
#         msg_blue.params = 'on=50% p=0.5 repeat=3n respawn=-1'
#         pub_blue.publish(msg_blue)
#     elif(s == 'LP'):
#         msg_blue.outputs = ['green_led']
#         msg_blue.type = 'TOGGLE'
#         msg_blue.params = 'on=50% p=1 repeat=3n respawn=-1'
#         pub_blue.publish(msg_blue)
#     elif(s == 'SSP'):
#         msg_blue.outputs = ['green_led']
#         msg_blue.type = 'TOGGLE'
#         msg_blue.params = 'on=50% p=0.2 repeat=2s respawn=-1'
#         pub_blue.publish(msg_blue)
#         msg_blue.outputs = ['blue_led']
#         msg_blue.type = 'TOGGLE'
#         msg_blue.params = 'on=75% p=0.45 repeat=2s respawn=-1'
#         pub_blue.publish(msg_blue)


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
        rospy.loginfo('Input params %s - %s - %d - %d - %d', name, topic, pin, bncTime, actLow)
        inputs.append(Input(name, topic, pin, bncTime, actLow))
        rospy.loginfo('Added input %s', name)
    
def AddOutput(config):
    global outputs
    name = config[0]
    pin = config[1]['pin']
    defSt = config[1]['default_state']

    if not(isinstance(pin, int) and isinstance(defSt, int)):
        rospy.loginfo('Unable to configure output %s. Bad parameter format.', name)
    else:
        outputs.append(Toggling_Output(name, pin, Fs, defSt))
        rospy.loginfo('Added output %s', name)


def NewOutputOrder(data):
    
    if(data.type == 'SET'):
        for o in outputs:
            if o.name in data.outputs:
                o.Set()
    elif(data.type == 'CLEAR'):
        for o in outputs:
            if o.name in data.outputs:
                o.Clear()
    else:
        # Tokenize params
        tok = list(filter(lambda a: False if (a=='' or a==' ' or a=='=') else True, re.split(r'([\d.]+|\W+)', data.params)))
        tok = [s.strip() for s in tok]
        tOn_u = tok[2]  # tOn units
        repeat_u = tok[7]
        respawn = tok[9]   # Respawn output if '=-'. 
        # rospy.loginfo('Output order tokens: %s', ' '.join(tok))

        try:
            tOn = float(tok[1])         # Time on / Duty cycle
            p = float(tok[4])           # Period
            repeat = float(tok[6])      # Repeat (for n seconds or n times)
            respawn_val = float(tok[10])  # Respawn delay
        except Exception:
            rospy.logwarn('%s -> Invalid arguments for output order', node_name)
            return

        for o in outputs:
            if o.name in data.outputs:
                o.ConfigToggle(time_on = tOn, time_on_unit = tOn_u, period = p, seq_length = repeat, length_unit = repeat_u, respawn = (-1 if respawn == '=-' else respawn_val))


