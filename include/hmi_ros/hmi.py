#!/usr/bin/env python

from pyrsistent import T
import rospy
import RPi.GPIO as GPIO
from .outputs import *
from .inputs import *
from msg import msg_outCmd

Fs = 50 # Hz
node_name = 'hmi_node'
enabled = False
inputs = []
outputs = []

#GPIO MAP
PIN_BTN_BLUE = 29
PIN_BTN_GREEN = 23
PIN_LED_BLUE = 31
PIN_LED_GREEN = 33
PIN_LED_COIL = 18

out_blue = None
in_blue = None
out_green = None
in_green = None
out_coil = None

def hmi_start():
    global out_blue, out_green, in_green, in_green, out_coil
    rospy.init_node('node_name')
    ConfigureHMI()

    pub = rospy.Publisher('gpio', String, queue_size=10)
    rate = rospy.Rate(Fs) # 10hz
    GPIO.setmode(GPIO.BOARD)

    # OUTPUT
    out_blue = Toggling_Output(name = 'out_blue', pin = PIN_LED_BLUE, sample_freq = Fs, default_state = 0)
    out_green = Toggling_Output(name = 'out_green', pin = PIN_LED_GREEN, sample_freq = Fs, default_state = 0)
    out_coil = Toggling_Output(name = 'out_coil', pin = PIN_LED_COIL, sample_freq = Fs, default_state = 0)

    out_blue.Config(time_on = 100, time_on_unit = '%', period = 0.8, seq_length = 4, length_unit = 's', respawn = 0)
    out_green.Config(time_on = 100, time_on_unit = '%', period = 0.8, seq_length = 4, length_unit = 's', respawn = 0)
    out_coil.Config(time_on = 50, time_on_unit = '%', period = 5, seq_length = 4, length_unit = 'n', respawn = 0)

    # INPUT
    in_blue = Input(name='in_blue', topicToPublish='hmi_node/inputs/in_blue', pin=PIN_BTN_BLUE, bouncetime = 10, activeLow = False)
    in_green = Input(name='in_green', topicToPublish='hmi_node/inputs/in_green', pin=PIN_BTN_GREEN, bouncetime = 10, activeLow = False)

    rospy.Subscriber('hmi_node/inputs/in_green', String, greenBtnCallback)
    while enabled and not rospy.is_shutdown():
        out_blue.Tick()
        out_green.Tick()
        out_coil.Tick()
        rate.sleep()

    del out_blue
    del out_green
    GPIO.cleanup()
    
def ConfigureHMI():
    global enabled, inputs, outputs

    if(rospy.has_param(node_name)):
        enabled = True
        Fs = rospy.get_param('sample_freq', 50)     # Default value is 50 Hz
        inputsConfig = (rospy.get_param('inputs'))  # Get list of dictionaries with the inputs from parameter server
        outputConfig = (rospy.get_param('outputs')) # Get list of dictionaries with the outputs from parameter server

        # Try to add each input and output
        for i in inputsConfig:
            AddInput(i)     
        for i in outputConfig:
            AddOutput(i)

    else:
        enabled = False
        rospy.logerr('No configuration parameter found. Parameter server should have parameter with same name as hmi node and include the inputs and outputs to configure.')

def AddInput(dict):
    
    
def AddOutput(dict):
    as

def greenBtnCallback(data):
    global out_green
    if(data.data == 'SP'):
        out_green.Restart()
    elif(data.data == 'SSP'):
        out_green.Config(time_on = 50, time_on_unit = '%', period = 0.35, seq_length = 4, length_unit = 'n')


def FinishedSeq(name):
    pass
    # out.Restart()

