#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from .outputs import *
from .inputs import *

Fs = 50 # Hz

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
    rospy.init_node('hmi_node')
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
    # GPIO.setup(18, GPIO.OUT)
    # GPIO.output(18, GPIO.HIGH)
    # INPUT
    in_blue = Input(name='in_blue', topicToPublish='hmi_node/inputs/in_blue', pin=PIN_BTN_BLUE, bouncetime = 10, activeLow = False)
    in_green = Input(name='in_green', topicToPublish='hmi_node/inputs/in_green', pin=PIN_BTN_GREEN, bouncetime = 10, activeLow = False)

    rospy.Subscriber('hmi_node/inputs/in_green', String, greenBtnCallback)
    while not rospy.is_shutdown():
        out_blue.Tick()
        out_green.Tick()
        out_coil.Tick()
        rate.sleep()

    del out_blue
    del out_green
    GPIO.cleanup()
    
def greenBtnCallback(data):
    global out_green
    if(data.data == 'SP'):
        out_green.Restart()
    elif(data.data == 'SSP'):
        out_green.Config(time_on = 50, time_on_unit = '%', period = 0.35, seq_length = 4, length_unit = 'n')


def FinishedSeq(name):
    pass
    # out.Restart()

