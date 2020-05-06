#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

cur_pose = None
path = None
driver_queue = None

DISTANCE_TOLERANCE = 0.1
ANGLE_TOLERANCE = 5

def cancel_path(_=None):
    global path
    path = None

def path_update(new_path):
    global path
    path = new_path

def adjust_path():
    global path

    if path is None or len(path) == 0:
        return

    if True: # x and y are within range
        path.pop(0)

    if True: # angle is within range
        driver_queue.put('fs')
    elif True: # angle is greater -> need to turn left
        driver_queue.put('sl')
    else: # angle is smaller -> need to turn right
        driver_queue.put('sr')

def update_pose(new_pose):
    global cur_pose
    cur_pose = new_pose
    adjust_path()

def setup_path_follow(queue=None):
    global driver_queue
    driver_queue = queue

    rospy.Subscriber('path', Path, path_updated)
    rospy.loginfo("subscribed to path")

    rospy.Subscriber('slam_out_pose', PoseStamped, update_pose)
    rospy.loginfo("subscribed to slam_out_pose")

if __name__ == "__main__":
    setup_path_follow()
