#!/usr/bin/env python

import subprocess
import rospy
import os
from remote_app.srv import LaunchRover, LaunchRoverResponse

running = {}

def doLaunch(req):
    rospy.loginfo("launching {} at {}".format(req.machine_name, req.ip_addr))
    try:
        #proc = subprocess.Popen(
        #    ['roslaunch', 'remote_app', 'rpi_nodes.launch',
        #     'machine_name:={}'.format(req.machine_name), 'ip_addr:={}'.format(req.ip_addr)])
        #running[req.machine_name] = proc
        return LaunchRoverResponse("")
    except e:
        rospy.logerr("failed to launch: {}".format(e))
        return LaunchRoverResponse(str(e))


def killRover(req):
    try:
        running[req.machine_name].terminate()
        return KillRoverResponse(True)
    except:
        return KillRoverResponse(False)


def setup_node():

    rospy.init_node('launcher');
    rospy.loginfo("starting")
    rospy.Service('/launch_rover', LaunchRover, doLaunch)
    rospy.loginfo("started /launch_rover service")
    rospy.spin()
    rospy.loginfo("stopping")


    # rospy.Subscriber('launch_rover', String, doLaunch)
    # rospy.Subscriber('kill_rover', String, killRover)
    #
    # rate = rospy.Rate(1)
    #
    # while not rospy.is_shutdown():
    #
    #     for machine_name in running.keys():
    #         status = running[machine_name].poll()
    #         if status is not None:
    #             print(status)
    #             del running[machine_name]
    #
    #     rate.sleep()

if __name__ == '__main__':
    setup_node()
