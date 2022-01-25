#!/usr/bin/env python

import rospy
from .outputs import *

Fs = 10 # Hz

# Board-specific. Implements the configuration of the pin
def ConfigPin(self, pin):
    #TBD
    return
    
# Board-specific. Implements the output set and clear routine
def SetOutput(pin, val):
    # self.board.digital_write(self.pin, val)
    rospy.loginfo('Setting pin %d to %d', pin, val)
    return

out = Toggling_Output(name = 'out1', pin = 1, pin_set_callback = SetOutput, sample_freq = Fs, default_state = 0)

def hmi_start():
    rospy.init_node('hmi_node')
    rate = rospy.Rate(Fs) # 10hz
    out.Config(time_on = 0.5, time_on_unit = 's', period = 1, seq_length = 4, length_unit = 'n', callback = FinishedSeq, respawn = 2)
    while not rospy.is_shutdown():
        out.Tick()
        rate.sleep()

def FinishedSeq(name):
    rospy.loginfo("HMI -> Finished sequence %s", name)
    # out.Restart()

