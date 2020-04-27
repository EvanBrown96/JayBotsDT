#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from jaybot.msg import Threshold

THRESHOLD = 0.25

threshold_queue = None

states = {"left_lidar": False,
          "right_lidar": False,
          "fwd_lidar": False
}

def scan_callback(scan_data):
    mins = {}
    mins["fwd_lidar"] = min(min(scan_data.ranges[:46]), min(scan_data.ranges[315:]))
    mins["left_lidar"] = min(scan_data.ranges[46:91])
    mins["right_lidar"] = min(scan_data.ranges[270:315])
    
    for s in states.keys():
        if (not states[s]) and mins[s] < THRESHOLD:
            threshold_queue.put(Threshold(s, True))
            states[s] = True
        elif states[s] and mins[s] >= THRESHOLD:
            threshold_queue.put(Threshold(s, False))
            states[s] = False

def start_lidar(t_q):
    global threshold_queue
    
    rospy.loginfo("LiDAR avoidance started")
    threshold_queue = t_q
    rospy.Subscriber('scan', LaserScan, scan_callback)
    rospy.spin()
    rospy.loginfo("LiDAR avoidance stopped")
    
