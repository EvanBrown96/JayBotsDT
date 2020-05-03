#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from jaybot.msg import Threshold

MINIMUM = 0.25
GOAL = 0.35
RECOVER = 0.40

threshold_comm = None

class Sensor:
    
    def __init__(self, name, threshold, calculation):
        self.name = name
        self.threshold = threshold
        self.calculation = calculation
        self.state = False

    def calculate(self, data):
        value = self.calculation(data)
        if (not self.state) and value < self.threshold:
            self.state = True
            return Threshold(self.name, True)
        elif self.state and value >= self.threshold:
            self.state = False
            return Threshold(self.name, False)
        return None

sensors = [
    Sensor("left_lidar", MINIMUM, lambda data: min(data.ranges[46:91])),
    Sensor("right_lidar", MINIMUM, lambda data: min(data.ranges[270:315])),
    Sensor("fwd_lidar", MINIMUM, lambda data: min(min(data.ranges[:46]), min(data.ranges[315:]))),
    Sensor("right_fwd_lidar", GOAL, lambda data: min(data.ranges[270:315])),
    Sensor("right_bck_lidar", RECOVER, lambda data: min(data.ranges[225:270]))
]

def scan_callback(scan_data):
    
    for s in sensors:
        result = s.calculate(scan_data)
        if result is not None:
            threshold_comm(result)


def start_lidar(threshold_queue=None):
    global threshold_comm

    rospy.loginfo("LiDAR avoidance starting")
    
    threshold_comm = lambda msg: threshold_queue.put(msg)

    if threshold_queue is None:
        rospy.init_node('lidar_avoidance')
        rospy.loginfo("started node")
        threshold_pub = rospy.Publisher('threshold', Threshold, queue_size=10)
        rospy.loginfo("publishing to threshold")
        threshold_comm = lambda msg: threshold_pub.publish(msg)
    
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.loginfo("subscribing to scan")

    rospy.spin()

    rospy.loginfo("LiDAR avoidance stopping")
    

if __name__ == "__main__":
    start_lidar()
