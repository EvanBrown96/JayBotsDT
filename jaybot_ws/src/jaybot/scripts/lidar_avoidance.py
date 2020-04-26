#!/usr/bin/env python

import rospy

THRESHOLD = 0.25

def start_lidar(thresh_queue):

    #rospy.init_node('sonar_array')

    left = SonarDevice('left', SONAR_LEFT_GPIO_TRIGGER, SONAR_LEFT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15, thresh_queue)
    right = SonarDevice('right', SONAR_RIGHT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15, thresh_queue)

    rate = rospy.Rate(40)
    rospy.loginfo("Sensors Started...")

    while not rospy.is_shutdown():
        left.scan()
        rate.sleep()

        right.scan()
        rate.sleep()

    thresh_queue.put(None)

    rospy.loginfo("Sensors Stopped")
