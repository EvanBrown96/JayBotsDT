#!/usr/bin/env python

from sonar_device import SonarDevice
import rospy

#-- RIGHT
SONAR_RIGHT_GPIO_TRIGGER     = 27
SONAR_RIGHT_GPIO_ECHO        = 22

#-- LEFT
SONAR_LEFT_GPIO_TRIGGER      = 23
SONAR_LEFT_GPIO_ECHO         = 24

MIN_RANGE = 0.02
MAX_RANGE = 2.0
THRESHOLD = 0.3

def start_sensors():

    rospy.init_node('sonar_array')

    left = SonarDevice('left', SONAR_LEFT_GPIO_TRIGGER, SONAR_LEFT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15)
    right = SonarDevice('right', SONAR_RIGHT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15)

    rate = rospy.Rate(20)
    rospy.loginfo("Sensors Started...")

    while not rospy.is_shutdown():
        left.scan()
        rate.sleep()

        right.scan()
        rate.sleep()

    rospy.loginfo("Sensors Stopped")


if __name__ == "__main__":

    start_sensors()