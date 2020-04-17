#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from threading import RLock
from sensor_msg.msg import LaserScan


class Mode:
    MANUAL = 0
    AUTONOMOUS = 1


mode = Mode.MANUAL
movement_state = 'stop'
vel_cmd_pub = None

left_avoiding = False
right_avoiding = False

callback_lock = RLock()


def commandCallback(user_command):
    global mode, movement_state

    cmd = user_command.data

    if cmd[0] == 'm':

        mode = Mode.MANUAL
        if cmd[2] == 'f':
            movement_state = 'forward'
        elif cmd[2] == 'b':
            movement_state = 'backward'
        elif cmd[3] == 'r':
            movement_state = 'right'
        elif cmd[3] == 'l':
            movement_state = 'left'
        else:
            movement_state = 'stop'

        callback_lock.acquire()
        if movement_state == 'forward' and (left_avoiding or right_avoiding):
            vel_cmd_pub.publish('stop')
        else:
            vel_cmd_pub.publish(movement_state)
        callback_lock.release()

    elif cmd[0] == 'a':

        mode = Mode.AUTONOMOUS
        autonomousSet()

def autonomousSet():
    callback_lock.acquire()
    if left_avoiding:
        vel_cmd_pub.publish('right')
    elif right_avoiding:
        vel_cmd_pub.publish('left')
    else:
        vel_cmd_pub.publish('forward')
    callback_lock.release()


def leftAvoidance(left_status):
    global left_avoiding

    callback_lock.acquire()

    left_avoiding = left_status.data

    if mode == Mode.MANUAL:
        if movement_state == 'forward' and not right_avoiding:
            if left_avoiding:
                vel_cmd_pub.publish('stop')
            else:
                vel_cmd_pub.publish(movement_state)

    elif mode == Mode.AUTONOMOUS:
        autonomousSet()

    callback_lock.release()


def rightAvoidance(right_status):
    global right_avoiding

    callback_lock.acquire()

    right_avoiding = right_status.data

    if mode == Mode.MANUAL:
        if movement_state == 'forward' and not left_avoiding:
            if right_avoiding:
                vel_cmd_pub.publish('stop')
            else:
                vel_cmd_pub.publish(movement_state)

    elif mode == Mode.AUTONOMOUS:
        autonomousSet()

    callback_lock.release()

def lidarAvoidance(lidar_status):
    callback_lock.acquire()
    zero_deg = lidar_status.ranges[359]
    forty_five_deg = lidar_status.ranges[314]
    ninety_deg = lidar_status.ranges[269]
    two_seventy_deg = lidar_status.ranges[89]
    three_fifteen_deg = lidar_status.ranges[44]
    if zero_deg <= .5 or forty_five_deg <= 0.5 or ninety_deg <= 0.5:
        right_avoiding = True
    else:
        right_avoiding = False

    if two_seventy_deg <= 0.5 or three_fifteen_deg <= 0.5:
        left_avoiding = True
    else:
        left_avoiding = False
    if mode == Mode.MANUAL:
        if movement_state == 'forward' and not right_avoiding:
            if left_avoiding:
                vel_cmd_pub.publish('stop')
            else:
                vel_cmd_pub.publish(movement_state)
        elif movement_state == 'forward' and not left_avoiding:
            if right_avoiding:
                vel_cmd_pub.publish('stop')
            else:
                vel_cmd_pub.publish(movement_state)
    elif mode == Mode.AUTONOMOUS:
        autonomousSet()
        
    callback_lock.release()


def setup_node():
    global vel_cmd_pub

    rospy.init_node('movement_logic')

    rospy.Subscriber('/jayrover/user_cmd', String, commandCallback)
    rospy.Subscriber('/jayrover/sonar/left_threshold', Bool, leftAvoidance)
    rospy.Subscriber('/jayrover/sonar/right_threshold', Bool, rightAvoidance)
    rospy.Subcriber('/jaybot/rplidar_ros/scan', LaserScan, lidarAvoidance)


    vel_cmd_pub = rospy.Publisher('/jayrover/vel_cmd', String, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    setup_node()
