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

MULTIPLIER = 0.25

#motors off
def motorsStop():
    left_spd.value = 0
    left_fwd.on()               #ignore this... they're actually being turned off
    left_bck.on()
    right_spd.value = 0
    right_fwd.on()
    right_bck.on()

#moving forward
def motorsFwd():
    left_spd.value = 1.1*MULTIPLIER
    left_fwd.on()
    left_bck.off()
    right_spd.value = 1*MULTIPLIER
    right_fwd.on()
    right_bck.off()

#moving backwards
def motorsBck():
    left_spd.value = 1*MULTIPLIER
    left_fwd.off()
    left_bck.on()
    right_spd.value = 1*MULTIPLIER
    right_fwd.off()
    right_bck.on()

#turning left
def motorsLeft():
    left_spd.value = 1*MULTIPLIER
    left_fwd.off()
    left_bck.on()
    right_spd.value = 1*MULTIPLIER
    right_fwd.on()
    right_bck.off()

#turning right
def motorsRight():
    left_spd.value = 1*MULTIPLIER
    left_fwd.on()
    left_bck.off()
    right_spd.value = 1*MULTIPLIER
    right_fwd.off()
    right_bck.on()

#message handler
def commandCallback(commandMessage):
    command = commandMessage.data
    if command == 'forward':
        rospy.loginfo('Moving Forward')
        motorsFwd()
    elif command == 'backward':
        rospy.loginfo('Moving Backward')
        motorsBck()
    elif command == 'left':
        rospy.loginfo('Turning Left')
        motorsLeft()
    elif command =='right':
        rospy.loginfo('Turning Right')
        motorsRight()
    elif command == 'stop':
        rospy.loginfo('Stopping')
        motorsStop()
    else:
        rospy.logwarn('Invalid command, so stopping instead')
        motorsStop()

def setup_node():

    rospy.init_node('driver')
    rospy.loginfo('starting node')
    rospy.Subscriber('vel_cmd', String, commandCallback)
    rospy.loginfo('subscribed to vel_cmd')
    rospy.spin()
    rospy.loginfo('Shutting down: shutting motors off')
    motorsStop()

if __name__ == '__main__':
    setup_node()
