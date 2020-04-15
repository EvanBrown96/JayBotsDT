#!/usr/bin/env python

import subprocess
import rospy
from remote_app.srv import LaunchRover, LaunchRoverResponse

running = {}


def doLaunch(req):
    rospy.loginfo(req)
    try:
        proc = subprocess.Popen(
            ['roslaunch', '~/JayBotsDT/launch/remote_standard.launch',
            'machine_name:={}'.format(req.machine_name), 'ip_addr:={}'.format(req.ip_addr)])
        running[req.machine_name] = proc
        return LaunchRoverResponse("")
    except e:
        return LaunchRoverResponse(str(e))


def killRover(req):
    try:
        running[req.machine_name].terminate()
        return KillRoverResponse(True)
    except:
        return KillRoverResponse(False)


def setup_node():

    rospy.init_node('launcher');
    rospy.Service('/launch_rover', LaunchRover, doLaunch)
    rospy.spin()


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
