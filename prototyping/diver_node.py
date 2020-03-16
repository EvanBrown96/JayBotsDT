#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import RPi.GPIO as GPIO

#set the GIPO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#set veraibles for the GPIO motor pins
motorPin_left_fwd = 20
motorPin_left_bck = 21
motorPin_right_fwd = 5
motorPin_right_bck = 6

#how many times to turn the pin on and off each second
frequency = 20

#how long the pin stays on each cycle, as a percent
dutyCyle = 30

#setting the duty cycle to 0 so that the motors don't run
stop = 0

GPIO.setup(motorPin_left_fwd, GPIO.OUT)
GPIO.setup(motorPin_left_bck, GPIO.OUT)
GPIO.setup(motorPin_right_fwd, GPIO.OUT)
GPIO.setup(motorPin_right_bck, GPIO.OUT)

#setting the GPIO's PWM frequency in hertz
motorPWM_left_fwd = GPIO.PWM(motorPin_left_fwd, frequency)
motorPWM_left_bck = GPIO.PWM(motorPin_left_bck, frequency)
motorPWM_right_fwd = GPIO.PWM(motorPin_right_fwd, frequency)
motorPWM_right_bck = GPIO.PWM(motorPin_right_bck, frequency)

#Lets start the software at a duty cycle of 0
motorPWM_left_fwd.start(stop)
motorPWM_left_bck.start(stop)
motorPWM_right_fwd.start(stop)
motorPWM_right_bck.start(stop)

#all motors off
def motorsStop():
    motorPWM_left_fwd.ChangeDutyCycle(stop)
    motorPWM_left_bck.ChangeDutyCycle(stop)
    motorPWM_right_fwd.ChangeDutyCycle(stop)
    motorPWM_right_bck.ChangeDutyCycle(stop)

#moving forward
def motorsFwd():
    motorPWM_left_fwd.ChangeDutyCycle(dutyCyle)
    motorPWM_left_bck.ChangeDutyCycle(stop)
    motorPWM_right_fwd.ChangeDutyCycle(dutyCyle)
    motorPWM_right_bck.ChangeDutyCycle(stop)

#moving backwards
def motorsBck():
    motorPWM_left_fwd.ChangeDutyCycle(stop)
    motorPWM_left_bck.ChangeDutyCycle(dutyCyle)
    motorPWM_right_fwd.ChangeDutyCycle(stop)
    motorPWM_right_bck.ChangeDutyCycle(dutyCyle)

#turning left
def motorsLeft():
    motorPWM_left_fwd.ChangeDutyCycle(stop)
    motorPWM_left_bck.ChangeDutyCycle(dutyCyle)
    motorPWM_right_fwd.ChangeDutyCycle(dutyCyle)
    motorPWM_right_bck.ChangeDutyCycle(stop)

#turning right
def motorsRight():
    motorPWM_left_fwd.ChangeDutyCycle(dutyCyle)
    motorPWM_left_bck.ChangeDutyCycle(stop)
    motorPWM_right_fwd.ChangeDutyCycle(stop)
    motorPWM_right_bck.ChangeDutyCycle(dutyCyle)

#message handler
def commandCallback(commandMessage):
    command = commandMessage.data
    if command == 'forward':
        print('Moving Forward')
        motorsFwd()
    elif command == 'backward':
        print('Moving Backward')
        motorsBck()
    elif command == 'left':
        print('Turning Left')
        motorsLeft()
    elif command =='right':
        print('Turning Right')
        motorsRight()
    elif command == 'stop':
        print('Stopping')
        motorsStop()
    else:
        print('Invalid command, so stopping instead')
        motorsStop()

rospy.init_node('driver')
rospy.Subscriber('command', String, commandCallback)
rospy.spin()
print('Shutting down: shutting motors off')
motorsStop()
GPIO.cleanup()
