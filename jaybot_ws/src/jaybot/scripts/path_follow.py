#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math
from tf.transformations import euler_from_quaternion

cur_pose = None
path = []
driver_queue = None
cancel = False

# tolerances for deciding if a position has been accurately matched
DISTANCE_TOLERANCE = 0.1
ANGLE_TOLERANCE = 10

# max time to move before rechecking position
MAX_PHASE_TIME = 1.0

# time to remain stationary and wait for slam position to stabilize after move
STATIONARY_TIME = 2.0

# velocity estimations, in degree/s and m/s
ang_vel_estimate = 45.0
linear_vel_estimate = 0.5
vel_sample_time = 0.0

def cancel_path(_=None):
    global cancel
    cancel = True

def path_update(new_path):
    global path
    path = new_path.poses
    rospy.loginfo("got updated path")

def follow_path():
    global path

    cancel = False
    last_pose = None
    phase_time = None
    while not cancel and len(path) > 0:
        this_pose = cur_pose

        x_off = path[0].pose.position.x - this_pose.pose.position.x
        y_off = path[0].pose.position.y - this_pose.pose.position.y

        if abs(x_off) < DISTANCE_TOLERANCE and abs(y_off) < DISTANCE_TOLERANCE:
            at_point = True

        if at_point:
            path.pop(0)
            if len(path) == 0:
                break
            # recalculate x_off and y_off
            x_off = path[0].pose.position.x - this_pose.pose.position.x
            y_off = path[0].pose.position.y - this_pose.pose.position.y

        cur_angle = math.degrees(euler_from_quaternion((this_pose.pose.orientation.x, this_pose.pose.orientation.y, this_pose.pose.orientation.z, this_pose.pose.orientation.w))[2])

        if last_pose is not None:
            # update velocity estimates based on distance moved and phase time
            distance_moved = math.sqrt(math.pow(this_pose.pose.position.x - last_pose.pose.position.x, 2) + math.pow(this_pose.pose.position.y - last_pose.pose.position.y, 2))
            last_angle = math.degrees(euler_from_quaternion((last_pose.pose.orientation.x, last_pose.pose.orientation.y, last_pose.pose.orientation.z, last_pose.pose.orientation.w))[2])
            angle_moved = abs((cur_angle - last_angle + 180) % 360 - 180) # math

            ang_vel_estimate = (ang_vel_estimate*vel_sample_time + angle_moved*phase_time)/(vel_sample_time+phase_time)
            linear_vel_estimate = (linear_vel_estimate*vel_sample_time + distance_moved*phase_time)/(vel_sample_time+phase_time)
            vel_sample_time += phase_time

        last_pose = cur_pose

        desired_angle = math.degrees(math.atan(y_off/(x_off + 0.0000001)))
        if x_off < 0:
            # account for atan only giving values between pi and -pi
            desired_angle += 180
        #rospy.loginfo("remaining steps: {}. goal: (x={} y={} angle={}), actual: (x={} y={} angle={})".format(len(path), path[0].pose.position.x, path[0].pose.position.y, desired_angle, this_pose.pose.position.x, this_pose.pose.position.y, actual_angle))
        angle_change = (desired_angle - cur_angle + 180) % 360 - 180 # math

        if abs(angle_change) < ANGLE_TOLERANCE: # angle is within range
            driver_queue.put('fs')
            pos_change = math.sqrt(math.pow(x_off, 2) + math.pow(y_off, 2))
            rospy.sleep(min(pos_change/linear_vel_estimate, MAX_PHASE_TIME))
        else:
            if angle_change > 0:
                driver_queue.put('sl')
            else:
                driver_queue.put('sr')
            rospy.sleep(min(angle_change/ang_vel_estimate, MAX_PHASE_TIME))

        driver_queue.put('ss')
        rospy.sleep(STATIONARY_TIME)


    # if path is None or len(path) == 0:
    #     return
    #
    # this_pose = cur_pose
    #
    # # determine if next point in path has been reached
    # at_point = False
    # if abs(path[0].pose.position.x - this_pose.pose.position.x) < DISTANCE_TOLERANCE and abs(path[0].pose.position.y - this_pose.pose.position.y) < DISTANCE_TOLERANCE:
    #     at_point = True
    #
    # if at_point:
    #     path.pop(0)
    #     if len(path) == 0:
    #         driver_queue.put("ss")
    #         return
    #
    # desired_angle = math.degrees(math.atan((path[0].pose.position.y - this_pose.pose.position.y)/(path[0].pose.position.x - this_pose.pose.position.x + 0.0000001)))
    # if path[0].pose.position.x - this_pose.pose.position.x < 0:
    #     # account for atan only giving values between pi and -pi
    #     desired_angle += 180
    # actual_angle = math.degrees(euler_from_quaternion((this_pose.pose.orientation.x, this_pose.pose.orientation.y, this_pose.pose.orientation.z, this_pose.pose.orientation.w))[2])
    # rospy.loginfo("remaining steps: {}. goal: (x={} y={} angle={}), actual: (x={} y={} angle={})".format(len(path), path[0].pose.position.x, path[0].pose.position.y, desired_angle, this_pose.pose.position.x, this_pose.pose.position.y, actual_angle))
    # angle_change = (desired_angle - actual_angle + 180) % 360 - 180 # math
    #
    # if abs(angle_change) < ANGLE_TOLERANCE: # angle is within range
    #     driver_queue.put('fs')
    # elif angle_change > 0: # angle is greater -> need to turn left
    #     driver_queue.put('sl')
    # else: # angle is smaller -> need to turn right
    #     driver_queue.put('sr')

def update_pose(new_pose):
    global cur_pose
    cur_pose = new_pose
    #adjust_path()

def setup_path_follow(queue=None):
    global driver_queue
    driver_queue = queue

    rospy.Subscriber('path', Path, path_update)
    rospy.loginfo("subscribed to path")

    rospy.Subscriber('slam_out_pose', PoseStamped, update_pose)
    rospy.loginfo("subscribed to slam_out_pose")

if __name__ == "__main__":
    setup_path_follow()
