#!/usr/bin/env python

from sonar_device import SonarDevice
import rospy
from jaybot.msg import Threshold

#-- RIGHT
SONAR_RIGHT_GPIO_TRIGGER     = 27
SONAR_RIGHT_GPIO_ECHO        = 22

#-- LEFT
SONAR_LEFT_GPIO_TRIGGER      = 23
SONAR_LEFT_GPIO_ECHO         = 24

MIN_RANGE = 0.02
MAX_RANGE = 2.0
THRESHOLD = 0.25

def start_sensors(thresh_queue):

    #rospy.init_node('sonar_array')

    left = SonarDevice('left_us', SONAR_LEFT_GPIO_TRIGGER, SONAR_LEFT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15, thresh_queue)
    right = SonarDevice('right_us', SONAR_RIGHT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15, thresh_queue)

    rate = rospy.Rate(40)
    rospy.loginfo("Sensors Started...")

    while not rospy.is_shutdown():
        left.scan()
        rate.sleep()

        right.scan()
        rate.sleep()

    rospy.loginfo("Sensors Stopped")
