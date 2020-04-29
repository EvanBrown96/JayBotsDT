#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gpiozero import DigitalOutputDevice, PWMOutputDevice, LED

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

MULTIPLIER = 0.2

spd_mappings = {
    "ss": (0, 0),
    "fs": (1.1, 1),
    "bs": (-1, -1),
    "sr": (1, -1),
    "fr": (1, 0),
    "br": (-1, 0),
    "sl": (-1, 1),
    "fl": (0, 1),
    "bl": (0, -1)
}


# #motors off
# def motorsStop():
#     left_spd.value = 0
#     left_fwd.on()               #ignore this... they're actually being turned off
#     left_bck.on()
#     right_spd.value = 0
#     right_fwd.on()
#     right_bck.on()

# #moving forward
# def motorsFwd():
#     left_spd.value = 1.1*MULTIPLIER
#     left_fwd.on()
#     left_bck.off()
#     right_spd.value = 1*MULTIPLIER
#     right_fwd.on()
#     right_bck.off()

# #moving backwards
# def motorsBck():
#     left_spd.value = 1*MULTIPLIER
#     left_fwd.off()
#     left_bck.on()
#     right_spd.value = 1*MULTIPLIER
#     right_fwd.off()
#     right_bck.on()

# #turning left
# def motorsLeft():
#     left_spd.value = 1*MULTIPLIER
#     left_fwd.off()
#     left_bck.on()
#     right_spd.value = 1*MULTIPLIER
#     right_fwd.on()
#     right_bck.off()

# #turning right
# def motorsRight():
#     left_spd.value = 1*MULTIPLIER
#     left_fwd.on()
#     left_bck.off()
#     right_spd.value = 1*MULTIPLIER
#     right_fwd.off()
#     right_bck.on()

#message handler
def commandCallback(command):
    left_velocity = spd_mappings[command.data][0]
    right_velocity = spd_mappings[command.data][1]

    left_spd.value = MULTIPLIER*abs(left_velocity)
    right_spd.value = MULTIPLIER*abs(right_velocity)

    if left_velocity > 0:
        left_fwd.on()
        left_bck.off()
    elif left_velocity < 0:
        left_fwd.off()
        left_bck.on()
    else:
        left_fwd.on()
        left_bck.on()

    if right_velocity > 0:
        right_fwd.on()
        right_bck.off()
    elif right_velocity < 0:
        right_fwd.off()
        right_bck.on()
    else:
        right_fwd.on()
        right_bck.on()


    # if command == 'forward':
    #     rospy.loginfo('Moving Forward')
    #     motorsFwd()
    # elif command == 'backward':
    #     rospy.loginfo('Moving Backward')
    #     motorsBck()
    # elif command == 'left':
    #     rospy.loginfo('Turning Left')
    #     motorsLeft()
    # elif command =='right':
    #     rospy.loginfo('Turning Right')
    #     motorsRight()
    # elif command == 'stop':
    #     rospy.loginfo('Stopping')
    #     motorsStop()
    # else:
    #     rospy.logwarn('Invalid command, so stopping instead')
    #     motorsStop()

def setup_driver(cmd_queue):

    rospy.loginfo('starting motor driver')
    while not rospy.is_shutdown():
        commandCallback(String(cmd_queue.get()))
    rospy.loginfo('Shutting down: shutting motors off')
    motorsStop()
