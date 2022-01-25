#!/usr/bin/env python

import rospy
from .outputs import *

Fs = 10 # Hz

out = Toggling_Output(name = 'out1', pin = 1, sample_freq = Fs, default_state = 0)

def hmi_start():
    rospy.init_node('hmi_node')
    rate = rospy.Rate(Fs) # 10hz
    out.Config(time_on = 50, time_on_unit = '%', period = 1, seq_length = 1.8, length_unit = 's', callback = FinishedSeq, respawn = 2)
    while not rospy.is_shutdown():
        out.Tick()
        rate.sleep()

def FinishedSeq(name):
    rospy.loginfo("HMI -> Finished sequence %s", name)
    # out.Restart()