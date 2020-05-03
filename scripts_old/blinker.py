#!/usr/bin/env python

from gpiozero import LED
import rospy
from std_srvs.srv import Empty, EmptyResponse

BLINKER_GPIO = 25
led = None

def start(_):
    led.blink(0.5, 2.5)
    return EmptyResponse()

def stop(_):
    led.off()
    return EmptyResponse()

def start_blinker():
    global led
    rospy.init_node('blinker')
    rospy.loginfo("starting node")

    rospy.Service("start_blinker", Empty, start)
    rospy.loginfo("started /start_blinker service")
    rospy.Service("stop_blinker", Empty, stop)
    rospy.loginfo("started /stop_blinker service")

    led = LED(BLINKER_GPIO)

    rospy.spin()

    rospy.loginfo("stopping node")


if __name__ == "__main__":

    start_blinker()
