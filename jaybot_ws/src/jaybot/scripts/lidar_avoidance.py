#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from jaybot.msg import Threshold

MINIMUM = 0.3
GOAL = 0.4

threshold_queue = None

class Sensor:
    
    def __init__(self, name, threshold, calculation):
        self.name = name
        self.threshold = threshold
        self.calculation = calculation
        self.state = False

    def calculate(self, data):
        return self.calculation(data)

sensors = [
    Sensor("left_lidar", MINIMUM, lambda data: min(data.ranges[46:91])),
    Sensor("right_lidar", MINIMUM, lambda data: min(data.ranges[270:315])),
    Sensor("fwd_lidar", MINIMUM, lambda data: min(min(data.ranges[:46]), min(data.ranges[315:]))),
    Sensor("right_fwd_lidar", GOAL, lambda data: min(data.ranges[270:315])),
    Sensor("right_bck_lidar", GOAL, lambda data: min(data.ranges[225:270]))
]

def scan_callback(scan_data):
    
    for s in sensors:
        value = s.calculate(scan_data)
        if (not s.state) and value < s.threshold:
            threshold_queue.put(Threshold(s.name, True))
            s.state = True
        elif s.state and value >= s.threshold:
            threshold_queue.put(Threshold(s.name, False))
            s.state = False

def start_lidar(t_q):
    global threshold_queue
    
    rospy.loginfo("LiDAR avoidance started")
    threshold_queue = t_q
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.spin()
    rospy.loginfo("LiDAR avoidance stopped")
    
