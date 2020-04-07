#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from threading import RLock


class Mode:
    MANUAL = 0


mode = Mode.MANUAL
movement_state = 'stop'
vel_cmd_pub = None

left_avoiding = False
right_avoiding = False

callback_lock = RLock()


def commandCallback(user_command):
    global mode

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


def leftAvoidance(left_status):
    global mode, left_avoiding
    callback_lock.acquire()

    left_avoiding = left_status.data

    if mode == Mode.MANUAL:
        if not right_avoiding:
            if left_status.data:
                vel_cmd_pub.publish('stop')
            elif movement_state == 'forward':
                vel_cmd_pub.publish(movement_state)

    callback_lock.release()


def rightAvoidance(right_status):
    global mode, right_avoiding
    callback_lock.acquire()

    right_avoiding = right_status.data

    if mode == Mode.MANUAL:
        if not left_avoiding:
            if right_status.data:
                vel_cmd_pub.publish('stop')
            elif movement_state == 'forward':
                vel_cmd_pub.publish(movement_state)

    callback_lock.release()


def setup_node():
    global vel_cmd_pub

    rospy.init_node('movement_logic')

    rospy.Subscriber('/jayrover/user_cmd', String, commandCallback)
    rospy.Subscriber('/jayrover/sonar/left_threshold', Bool, leftAvoidance)
    rospy.Subscriber('/jayrover/sonar/right_threshold', Bool, rightAvoidance)

    vel_cmd_pub = rospy.Publisher('/jayrover/vel_cmd', String, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    setup_node()
