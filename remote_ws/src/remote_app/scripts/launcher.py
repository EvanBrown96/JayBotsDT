#!/usr/bin/env python

import subprocess
import rospy
import os
from remote_app.srv import LaunchRover, LaunchRoverResponse, PollRover, PollRoverResponse, KillRover, KillRoverResponse
import StringIO

running = {}

def doLaunch(req):
    rospy.loginfo("launching {} at {}".format(req.machine_name, req.ip_addr))
    try:
        err = StringIO.StringIO()
        proc = subprocess.Popen(
            ['roslaunch', 'remote_app', 'rpi_nodes.launch',
             'machine_name:={}'.format(req.machine_name), 'ip_addr:={}'.format(req.ip_addr)],
            stderr=err)
        running[req.machine_name] = (proc, err)
        return LaunchRoverResponse("")
    except e:
        rospy.logerr("failed to launch: {}".format(e))
        return LaunchRoverResponse(str(e))

def doPoll(req):

    return KillRoverResponse(running[req.machine_name][1].getvalue())

def doKill(req):
    running[req.machine_name][0].terminate()
    running[req.machine_name][1].close()
    return KillRoverResponse()


def setup_node():

    rospy.init_node('launcher');
    rospy.loginfo("starting node")

    rospy.Service('/launch_rover', LaunchRover, doLaunch)
    rospy.loginfo("started /launch_rover service")
    rospy.Service('/poll_rover', PollRover, doPoll)
    rospy.loginfo("started /poll_rover service")
    rospy.Service('/kill_rover', KillRover, doKill)

    rospy.spin()
    rospy.loginfo("stopping node")


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
