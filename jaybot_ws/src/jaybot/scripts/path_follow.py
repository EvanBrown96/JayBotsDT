#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

cur_pose = None
path = None

def cancel_path(_=None):
    global path
    path = None

def path_update(new_path):
    global path
    path = new_path

def update_pose(new_pose):
    global cur_pose
    cur_pose = new_pose

def start_path(_=None):
    global path
    while path is not None and len(path.points) > 0:
        path.pop()


def setup_path_follow(driver_queue=None):

    rospy.Subscriber('path', Path, path_updated)
    rospy.loginfo("subscribed to path")

    rospy.Subscriber('slam_out_pose', PoseStamped, update_pose)
    rospy.loginfo("subscribed to slam_out_pose")

if __name__ == "__main__":
    setup_path_follow()
