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


def start_sensors(thresh_queue=None):

    rospy.loginfo("starting sensors")
    
    sensors = [
        SonarDevice('left_us', SONAR_LEFT_GPIO_TRIGGER, SONAR_LEFT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15),
        SonarDevice('right_us', SONAR_RIGHT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_ECHO, MIN_RANGE, THRESHOLD, MAX_RANGE, -15, 15)
    ]

    rate = rospy.Rate(40)
    
    threshold_comm = lambda msg: thresh_queue.put(msg)

    if thresh_queue is None:
        rospy.init_node('avoidance')
        rospy.loginfo("started node")
        threshold_pub = rospy.Publisher('threshold', Threshold, queue_size=10)
        rospy.loginfo("publishing to threshold")
        threshold_comm = lambda msg: threshold_pub.publish(msg)

    while not rospy.is_shutdown():
        for s in sensors:
            result = s.scan()
            if result is not None:
                threshold_comm(result)
        rate.sleep()

    rospy.loginfo("stopping sensors")


if __name__ == "__main__":
    start_sensors()
