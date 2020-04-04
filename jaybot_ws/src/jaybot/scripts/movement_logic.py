#!/usr/bin/env python

import rospy
from std_msgs.msg import String

class Mode:
    MANUAL = 0


mode = Mode.MANUAL
vel_cmd_pub = None


def commandCallback(user_command):
    global mode

    cmd = user_command.data

    if cmd[0] == 'm':
        mode = Mode.MANUAL
        if cmd[2] == 'f':
            vel_cmd_pub.publish('forward')
        elif cmd[2] == 'b':
            vel_cmd_pub.publish('backward')
        elif cmd[3] == 'r':
            vel_cmd_pub.publish('right')
        elif cmd[3] == 'l':
            vel_cmd_pub.publish('left')
        else:
            vel_cmd_pub.publish('stop')


def setup_node():
    global vel_cmd_pub

    rospy.init_node('movement_logic')

    rospy.Subscriber('user_cmd', String, commandCallback)

    vel_cmd_pub = rospy.Publisher('vel_cmd', String, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    setup_node()
