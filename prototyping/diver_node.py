#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gpiozero import DigitalOutputDevice, PWMOutputDevice

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

#motors off
def motorsStop():
    left_spd.value = 0
    left_fwd.off()
    left_bck.off()
    right_spd.value = 0
    right_fwd.off()
    right_bck.off()

#moving forward
def motorsFwd():
    left_spd.value = 0.5
    left_fwd.on()
    left_bck.off()
    right_spd.value = 0.5
    right_fwd.on()
    right_bck.off()

#moving backwards
def motorsBck():
    left_spd.value = 0.5
    left_fwd.off()
    left_bck.on()
    right_spd.value = 0.5
    right_fwd.off()
    right_bck.on()

#turning left
def motorsLeft():
    left_spd.value = 0.5
    left_fwd.off()
    left_bck.on()
    right_spd.value = 0.5
    right_fwd.on()
    right_bck.off()

#turning right
def motorsRight():
    left_spd.valeu = 0.5
    left_fwd.on()
    left_bck.off()
    right_spd.value = 0.5
    right_fwd.off()
    right_bck.on()

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
