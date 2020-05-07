#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
from tf.transformations import euler_from_quaternion

cur_pose = None
path = None
driver_queue = None

DISTANCE_TOLERANCE = 0.05
ANGLE_TOLERANCE = 3

def cancel_path(_=None):
    global path
    path = None

def path_update(new_path):
    global path
    path = new_path.poses

def adjust_path():
    global path

    if path is None or len(path) == 0:
        return

    # determine if next point in path has been reached
    at_point = False
    if abs(path[0].pose.point.x - cur_pose.pose.point.x) < DISTANCE_TOLERANCE and abs(path[0].pose.point.y - cur_pose.pose.point.y) < DISTANCE_TOLERANCE:
        at_point = True

    if at_point:
        path.pop(0)

    desired_angle = math.degrees(math.atan((path[0].pose.point.y - cur_pose.pose.point.y)/(path[0].pose.point.x - cur_pose.pose.point.x + 0.0000001)))
    actual_angle = math.degrees(euler_from_quaternion((cur_pose.pose.orientation.x, cur_pose.pose.orientation.y, cur_pose.pose.orientation.z, cur_pose.pose.orientation.w))[2])
    angle_change = (desired_angle - actual_angle + 180) % 360 - 180 # math

    if abs(angle_change) < ANGLE_TOLERANCE: # angle is within range
        driver_queue.put('fs')
    elif angle_change > 0: # angle is greater -> need to turn left
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

    rospy.Subscriber('path', Path, path_update)
    rospy.loginfo("subscribed to path")

    rospy.Subscriber('slam_out_pose', PoseStamped, update_pose)
    rospy.loginfo("subscribed to slam_out_pose")

if __name__ == "__main__":
    setup_path_follow()
