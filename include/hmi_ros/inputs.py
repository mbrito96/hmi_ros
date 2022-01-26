#!/usr/bin/env python

# import RPi.GPIO as GPIO		# Jetson GPIO is analogue to RPi GPIO library. Use RPi for portability
import rospy
from std_msgs.msg import String

TYPE_NONE = 0
TYPE_SHORT_PRESS = 1
TYPE_LONG_PRESS = 2

T_SHORT_PRESS = 1.5
T_LONG_PRESS = 4 # sec
T_FOLLOWUP_WINDOW = 1.5	# After off edge (on -> off), window of time for the user to input another button to be considered in the command
T_CANCEL_WINDOW = 3	# After on edge (off -> on), window of time before aborting the command

STATE_IDLE = 0
STATE_DETECTING = 1

class GPIO():
	def __init__(self):
		self.LOW = 0
		self.HIGH = 1


class Input:
	def __init__(self, name, pin, bouncetime = 10, activeLow = False):
		self.name = name
		self.pin = pin
		self.bouncetime = bouncetime
		self.activeLow = activeLow
		self.input_buf = []
		self.time_edgeOn = 0.0
		self.time_edgeOff = 0.0
		self.actionTimer = None
		self.state = STATE_IDLE
		# GPIO.setmode(GPIO.BOARD)
		# GPIO.setup(pin, GPIO.IN)
		# GPIO.add_event_detect(pin, GPIO.BOTH, callback=GpioCallback, bouncetime=10)
		self.subs = rospy.Subscriber('gpio', String, self.GpioCallback)


	def GpioCallback(self, channel):
		# val = GPIO.input(channel)
		if(channel.data == '0'):
			val = GPIO().LOW
		else:
			val = GPIO().HIGH

		rospy.loginfo('%s -> Input %d changed to %d', self.name, self.pin, val)
		
		edgeOn = False			# The edge detected occured from off to on state
		if(val == GPIO().HIGH): 	# Check current input state 
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

	def CheckForSequenceDone(self):
		retVal = True
		if(self.input_buf == [TYPE_SHORT_PRESS]):
			rospy.loginfo('%s -> ==== SHORT PRESS ====.', self.name)
		elif(self.input_buf == [TYPE_SHORT_PRESS, TYPE_SHORT_PRESS]):
			rospy.loginfo('%s -> ==== DOUBLE SHORT PRESS ====.', self.name)
		elif(self.input_buf == [TYPE_LONG_PRESS]):
			rospy.loginfo('%s -> ==== LONG PRESS ====.', self.name)
		elif(self.input_buf == [TYPE_SHORT_PRESS, TYPE_SHORT_PRESS, TYPE_LONG_PRESS]):
			rospy.loginfo('%s -> ==== DOUBLE SHORT SINGLE LONG PRESS ====.', self.name)
		else:
			rospy.loginfo('%s -> ==== NONE ====.', self.name)
			retVal = False
		return retVal


	def ChangeState(self, st):
		if(st == STATE_IDLE):
			rospy.loginfo('Going idle...')	
			self.state = STATE_IDLE
			self.actionTimer.shutdown()
		elif(st == STATE_DETECTING):
			rospy.loginfo('Detecting...')
			self.input_buf.clear()
			self.state = STATE_DETECTING


	def TimerCancelCallback(self, *args):
		rospy.loginfo('%s -> Cancelling input...', self.name)
		self.ChangeState(STATE_IDLE)
		
	def TimerFollowupCallback(self, *args):
		self.CheckForSequenceDone()	# If valid sequence detected in input buffer
		self.ChangeState(STATE_IDLE)

