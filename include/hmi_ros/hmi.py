#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from .outputs import *
from .inputs import *

Fs = 10 # Hz

# Board-specific. Implements the configuration of the pin
def ConfigPin(self, pin):
    #TBD
    return
    
# Board-specific. Implements the output set and clear routine
def SetOutput(pin, val):
    GPIO.output(pin, GPIO.HIGH if val == 1 else GPIO.LOW)
    # rospy.loginfo('Setting pin %d to %d', pin, val)
    return

out1 = None
in1 = None

def hmi_start():
    global out1
    rospy.init_node('hmi_node')
    pub = rospy.Publisher('gpio', String, queue_size=10)
    rate = rospy.Rate(Fs) # 10hz
    GPIO.setmode(GPIO.BOARD)
    # OUTPUT
    outPin = 31
    GPIO.setup(outPin, GPIO.OUT, initial=GPIO.LOW)
    out1 = Toggling_Output(name = 'out1', pin = outPin, pin_set_callback = SetOutput, sample_freq = Fs, default_state = 0)
    out1.Config(time_on = 50, time_on_unit = '%', period = 0.8, seq_length = 4, length_unit = 's', callback = FinishedSeq, respawn = 2)
    # INPUT
    in1 = Input(name='IN1', pin=12, bouncetime = 10, activeLow = False)

    while not rospy.is_shutdown():
    #    print("Waiting for key...")
    #    inp = input()
    #    pub.publish(inp)
        # out1.Tick()
        rate.sleep()
    GPIO.cleanup()
    

def FinishedSeq(name):
    rospy.loginfo("HMI -> Finished sequence %s", name)
    # out.Restart()

