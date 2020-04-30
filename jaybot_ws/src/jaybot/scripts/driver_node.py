#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from gpiozero import DigitalOutputDevice, PWMOutputDevice, LED
from jaybot.srv import SetSpeed

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

multip = 0.2

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


#message handler
def commandCallback(command):
    if command.data not in spd_mappings.keys():
        rospy.logwarn("invalid movement command!!!")
        return

    left_velocity = spd_mappings[command.data][0]
    right_velocity = spd_mappings[command.data][1]

    left_spd.value = multiplier*abs(left_velocity)
    right_spd.value = multiplier*abs(right_velocity)

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


def speed_callback(speed_cmd):
    global multiplier
    multiplier = speed_cmd.spd_pct/100


def driver_setup(cmd_queue=None):

    rospy.loginfo('starting motor driver')
    
    rospy.Service('set_speed', SetSpeed, speed_callback)

    if cmd_queue is None:
        rospy.init_node('driver')
        rospy.loginfo("started node")
        rospy.Subscriber('vel_cmd', String, commandCallback)
        rospy.loginfo("subscribed to vel_cmd")
        rospy.spin()
    else:
        while not rospy.is_shutdown():
            commandCallback(String(cmd_queue.get()))

    rospy.loginfo('stopping motor driver')


if __name__ == "__main__":
    driver_setup()
