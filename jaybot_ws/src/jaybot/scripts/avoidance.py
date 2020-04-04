#!/usr/bin/env python

from gpiozero import DistanceSensor
import math
import rospy
from sensor_msgs.msg import Range

#-- RIGHT
SONAR_RIGHT_GPIO_TRIGGER     = 27
SONAR_RIGHT_GPIO_ECHO        = 22

#-- LEFT
SONAR_LEFT_GPIO_TRIGGER      = 23
SONAR_LEFT_GPIO_ECHO         = 24

class SonarDevice():

    def __init__(self, name, trigger, echo, range_min, threshold, range_max, angle_min_deg, angle_max_deg):
        self.name = name
        self.range_min = range_min
        self.range_max = range_max
        self.fov = math.radians(angle_max_deg-angle_min_deg)
        self.sensor = DistanceSensor(echo=echo, trigger=trigger, max_distance=range_max)

        topic_name = "/jayrover/sonar/{}".format(name)
        self.pub = rospy.Publisher(topic_name, Range, queue_size=1)
        rospy.loginfo("Publisher set with topic {}".format(topic_name))

    def scan(self):

        range_val = self.sensor.distance
        
        message = Range()
        message.radiation_type = 0
        message.field_of_view = self.fov
        message.min_range = self.range_min
        message.max_range = self.range_max
        message.range = range_val

        self.pub.publish(message)

        rospy.loginfo("Range {} [m]: {}".format(self.name, range_val))


def start_sensors():

    rospy.init_node('sonar_array')

    left = SonarDevice('left', SONAR_LEFT_GPIO_TRIGGER, SONAR_LEFT_GPIO_ECHO, 0.02, 0.3, 2.0, -15, 15)
    right = SonarDevice('right', SONAR_RIGHT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_ECHO, 0.02, 0.3, 2.0, -15, 15)

    rate = rospy.Rate(20)
    rospy.loginfo("Sensors Running...")

    while not rospy.is_shutdown():
        left.scan()
        rate.sleep()

        right.scan()
        rate.sleep()

    rospy.loginfo("Sensors Stopped")


if __name__ == "__main__":

    start_sensors()