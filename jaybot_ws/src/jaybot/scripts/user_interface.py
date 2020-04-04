#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import String

PORT = 10001

def start_node():
    sock = socket.socket()
    sock.bind(('', PORT))
    sock.listen()

    pub = rospy.Publisher('user_cmd', String)

    while not rospy.is_shutdown():
        conn, _ = sock.accept()

        try:
            while not rospy.is_shutdown():

                data = conn.recv(4).decode('utf-8')

                if len(data) == 0:
                    break
            
                pub.publish(data)

            conn.shutdown(socket.SHUT_RDWR)

        except Exception as e:
            rospy.logwarn(e)
        
        conn.close()

if __name__ == '__main__':
    start_node()