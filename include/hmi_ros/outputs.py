#!/usr/bin/env python

import RPi.GPIO as GPIO
import rospy

class Toggling_Output:

    # Board-specific. Implements the configuration of the pin
    def ConfigGpio(self):
        GPIO.setup(self.pin, GPIO.OUT, initial=self.default_state)
        return

    # Board-specific. Implements the output set and clear routine
    def SetOutput(self, val):
        GPIO.output(self.pin, GPIO.HIGH if val == 1 else GPIO.LOW)
        # rospy.loginfo('Setting pin %d to %d', pin, val)
        return

    # default_state 0 or 1. The value to set the output when the sequence finishes 
    def __init__(self, name, pin, sample_freq, default_state = 0):
        self.name = name
        self.pin = pin
        self.enabled = False
        self.Fs = sample_freq
        self.default_state = default_state
        self.ConfigGpio()
        self.SetOutput(self.default_state)
    
    def __del__(self):
        self.SetOutput(self.default_state)

    # @param time_on_unit Defines whether time_on is given in seconds ('s') or as duty cycle ('%')
    # @param length_unit Defines whether length is given in seconds ('s') or as number of repetitions ('n')
    # @param period In seconds, the period of the toggle (on time + off time)
    # @param seq_length If length_unit is 's' -> Number of seconds to repeat the toggling sequence ()
    #                   If length_unit is 'n' -> Number of periods to repeat the toggling sequence 
    # @param time_on If time_on_unit is 's' -> Seconds to keep the output high (must be smaller than period)
    #                If time_on_unit is '%' -> Percentage of period to keep the output high 
    # @param callback Function for calling when sequence finishes. Name of output is sent as parameter
    # @param respawn Delay in seconds after output sequence finishes for restarting the sequence. -1 disables respawn feature
    def Config(self, time_on, time_on_unit, period, seq_length, length_unit, callback = None, respawn = -1):
        self.enabled = True
        self.finished = False
        self.togCount = 0      # to keep track of number of ticks elapsed
        self.seqCount = 0      # to keep track of number of ticks elapsed
        self.tPeriod = period * self.Fs
        self.Callback = callback
        self.respawn = False
        if(respawn >= 0):
           self.respawn = True
           self.respawn_delay = respawn * self.Fs

        if(time_on_unit == 's'):
            self.tOn = time_on * self.Fs
        elif(time_on_unit == '%'):
            self.tOn = time_on * self.tPeriod / 100     # Convert percentage to ticks
        else:
            self.enabled = False

        if(length_unit == 's'):
            self.tSequence = seq_length * self.Fs
        elif(length_unit == 'n'):
            self.tSequence = int(seq_length) * self.tPeriod
        else:
            self.enabled = False

        if(self.tOn > self.tPeriod or self.tPeriod > self.tSequence):     # Basic coherence check
            self.enabled = False
            rospy.logwarn("%s: Invalid output configuration.", self.name)

    def Restart(self):
        self.togCount = 0      # to keep track of number of ticks elapsed
        self.seqCount = 0      # to keep track of number of ticks elapsed
        self.finished = False

    def Tick(self):
        if(self.enabled == False):
            return


        if(self.finished == False):
            self.seqCount+=1   
            self.togCount+=1
            
            if(self.seqCount >= self.tSequence):  # Sequence finished, terminate
                self.seqCount = 0      # Also used as respawn counter
                self.finished = True
                self.SetOutput(self.default_state) 
                if(self.Callback != None):
                    self.Callback(self.name)
            else:                       # Else, execute toggling logic
                if(self.togCount <= self.tOn):
                    self.SetOutput(1)
                elif(self.togCount < self.tPeriod):
                    self.SetOutput(0)
                
                if(self.togCount >= self.tPeriod):
                    self.togCount = 0
        elif(self.respawn == True):
            self.seqCount+=1     
            if(self.seqCount >= self.respawn_delay):
                self.Restart()
