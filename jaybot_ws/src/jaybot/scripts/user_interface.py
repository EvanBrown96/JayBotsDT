#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import String

PORT = 10001

def start_node():
    sock = socket.socket()
    sock.bind(('', PORT))
    sock.listen(0)

    rospy.init_node('user_interface')
    pub = rospy.Publisher('/jayrover/user_cmd', String, queue_size=10)

    while not rospy.is_shutdown():
        conn, _ = sock.accept()
        saved = ""

        try:
            while not rospy.is_shutdown():

                current = conn.recv(1024).decode('utf-8')
                if not current:
                    break

                saved += current
                if len(saved < 4):
                    continue

                pub.publish(saved[:4])
                saved = saved[4:]

            conn.shutdown(socket.SHUT_RDWR)

        except Exception as e:
            rospy.logwarn(e)

        conn.close()

if __name__ == '__main__':
    start_node()
