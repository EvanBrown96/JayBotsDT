#!/usr/bin/env python

from gpiozero import LED
import rospy
from std_srvs.srv import Empty, EmptyResponse

#-- RIGHT
BLINKER_GPIO = 25
led = None

def start(Empty e):
    led.blink()

def stop(Empty e):
    led.off()

def start_blinker():
    global led
    rospy.init_node('blinker')
    rospy.loginfo("starting node")

    rospy.Service("/start_blinker", Empty, start)
    rospy.loginfo("started /start_blinker service")
    rospy.Service("/stop_blinker", Empty, stop)
    rospy.loginfo("started /stop_blinker service")

    led = LED(BLINKER_GPIO)

    rospy.spin()

    # rate = rospy.Rate(1)
    #
    # while not rospy.is_shutdown():
    #     rate.sleep()
    #     led.
    #     if blinking:


    rospy.loginfo("stopping node")


if __name__ == "__main__":

    start_blinker()
