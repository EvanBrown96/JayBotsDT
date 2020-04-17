#!/usr/bin/env python

import subprocess
import rospy
import os
from remote_app.srv import LaunchRover, LaunchRoverResponse, PollRover, PollRoverResponse, KillRover, KillRoverResponse


running = {}


def doLaunch(req):
    rospy.loginfo("launching {} at {}".format(req.machine_name, req.ip_addr))
    try:
        proc = subprocess.Popen(
            ['roslaunch', 'remote_app', 'rpi_nodes.launch',
             'machine_name:={}'.format(req.machine_name),
             'ip_addr:={}'.format(req.ip_addr)],
             stderr=subprocess.PIPE)
        running[req.machine_name] = proc
        return LaunchRoverResponse("")

    except Exception as e:
        rospy.logerr("failed to launch: {}".format(e))
        return LaunchRoverResponse(str(e))


def doPoll(req):

    if req.machine_name not in running.keys():
        return PollRoverResponse(False, "")

    if running[req.machine_name].poll() is not None:
        out, err = running[req.machine_name].communicate()
        del running[req.machine_name]
        return PollRoverResponse(False, err)

    return PollRoverResponse(True, "")


def doKill(req):
    if req.machine_name in running.keys():
        running[req.machine_name].terminate()
        del running[req.machine_name]
    return KillRoverResponse()


def setup_node():

    rospy.init_node('launcher');
    rospy.loginfo("starting node")

    rospy.Service('/launch_rover', LaunchRover, doLaunch)
    rospy.loginfo("started /launch_rover service")
    rospy.Service('/poll_rover', PollRover, doPoll)
    rospy.loginfo("started /poll_rover service")
    rospy.Service('/kill_rover', KillRover, doKill)
    rospy.loginfo("started /kill_rover service")

    rospy.spin()
    rospy.loginfo("stopping node")


if __name__ == '__main__':
    setup_node()
