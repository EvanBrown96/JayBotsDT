
from gpiozero import DistanceSensor
import rospy
from jaybot.msg import Threshold
import math

class SonarDevice():

    def __init__(self, name, trigger, echo, range_min, threshold, range_max, angle_min_deg, angle_max_deg):
        self.name = name
        self.range_min = range_min
        self.range_max = range_max
        self.threshold = threshold
        self.fov = math.radians(angle_max_deg-angle_min_deg)
        self.sensor = DistanceSensor(echo=echo, trigger=trigger, max_distance=range_max)
        self.threshold_state = False

    def scan(self):

        range_val = self.sensor.distance

        if (not self.threshold_state) and range_val < self.threshold:
            self.threshold_state = True
            rospy.logdebug("Range {} [m]: {} (in threshold: {})".format(self.name, range_val, self.threshold_state))
            return Threshold(self.name, True)
        elif self.threshold_state and range_val >= self.threshold:
            self.threshold_state = False
            rospy.logdebug("Range {} [m]: {} (in threshold: {})".format(self.name, range_val, self.threshold_state))
            return Threshold(self.name, False)

        return None

