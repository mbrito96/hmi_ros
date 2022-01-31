#!/usr/bin/env python

import RPi.GPIO as GPIO		# Jetson GPIO is analogue to RPi GPIO library. Use RPi for portability
import rospy
from std_msgs.msg import String

TYPE_NONE = 0
TYPE_SHORT_PRESS = 1
TYPE_LONG_PRESS = 2

T_SHORT_PRESS = 0.8
T_LONG_PRESS = 2 # sec
T_FOLLOWUP_WINDOW = 1.5	# After off edge (on -> off), window of time for the user to input another button to be considered in the command
T_CANCEL_WINDOW = 4	# After on edge (off -> on), window of time before aborting the command

STATE_IDLE = 0
STATE_DETECTING = 1

class Input:
	# @param topicToPublish name of the topic to publish input detected. Message type must be std_msgs.msg.String
	def __init__(self, name, topicToPublish, pin, bouncetime = 10, activeLow = False):
		self.name = name
		self.pin = pin
		self.bouncetime = bouncetime
		self.activeLow = activeLow
		self.input_buf = []
		self.time_edgeOn = 0.0
		self.time_edgeOff = 0.0
		self.actionTimer = None
		self.state = STATE_IDLE
		GPIO.setup(pin, GPIO.IN)
		GPIO.add_event_detect(pin, GPIO.BOTH, callback=self.GpioCallback, bouncetime=50)
		self.pub = rospy.Publisher(topicToPublish, String, queue_size=10)


	def GpioCallback(self, channel):
		val = GPIO.input(channel)
		
		edgeOn = False			# The edge detected occured from off to on state
		if(val == GPIO.HIGH): 	# Check current input state 
			edgeOn = True
		if(self.activeLow):		# Invert detection when configured as active low 	
			edgeOn = not edgeOn
		
		if(edgeOn):
			if(self.state == STATE_IDLE):	# Change to DETECTING state
				self.ChangeState(STATE_DETECTING)

			self.time_edgeOn = rospy.Time.now()
			try:
				self.actionTimer.shutdown()
			except Exception:
				pass
			self.actionTimer = rospy.Timer(rospy.Duration(T_CANCEL_WINDOW), self.TimerCancelCallback, True) 
		else:
			if(self.state == STATE_DETECTING):
				self.time_edgeOff = rospy.Time.now()
				self.input_buf += filter(None, [self.ComputeInput()])	# Add value (if valid) to input buffer
				
				try:
					self.actionTimer.shutdown()
				except Exception:
					pass
				self.actionTimer = rospy.Timer(rospy.Duration(T_FOLLOWUP_WINDOW), self.TimerFollowupCallback, True) 


	def ComputeInput(self):
		timeDiff = (self.time_edgeOff - self.time_edgeOn).to_sec()
		if(timeDiff < T_SHORT_PRESS):
			return TYPE_SHORT_PRESS
		elif(timeDiff < T_LONG_PRESS):
			return TYPE_LONG_PRESS
		else:
			return None

	def IdentifySequence(self):
		detected = True
		valDetected = ""
		if(self.input_buf == [TYPE_SHORT_PRESS]):
			rospy.logdebug('%s -> ==== SHORT PRESS ====.', self.name)
			valDetected = 'SP'	# Short press
		elif(self.input_buf == [TYPE_SHORT_PRESS, TYPE_SHORT_PRESS]):
			rospy.logdebug('%s -> ==== DOUBLE SHORT PRESS ====.', self.name)
			valDetected = 'SSP'	# Double short press
		elif(self.input_buf == [TYPE_LONG_PRESS]):
			rospy.logdebug('%s -> ==== LONG PRESS ====.', self.name)
			valDetected = 'LP'	# Long press
		elif(self.input_buf == [TYPE_SHORT_PRESS, TYPE_SHORT_PRESS, TYPE_LONG_PRESS]):
			rospy.logdebug('%s -> ==== DOUBLE SHORT SINGLE LONG PRESS ====.', self.name)
			valDetected = 'SSLP'	# Double short Long press
		else:
			rospy.logdebug('%s -> ==== NONE ====.', self.name)
			detected = False
		
		if(detected):
			self.pub.publish(valDetected)

		return detected


	def ChangeState(self, st):
		if(st == STATE_IDLE):
			self.state = STATE_IDLE
			try:
				self.actionTimer.shutdown()
			except Exception:
				pass
		elif(st == STATE_DETECTING):
			self.input_buf *= 0	# clear() method not working
			self.state = STATE_DETECTING


	def TimerCancelCallback(self, *args):
		self.ChangeState(STATE_IDLE)
		
	def TimerFollowupCallback(self, *args):
		self.IdentifySequence()	# If valid sequence detected in input buffer
		self.ChangeState(STATE_IDLE)

