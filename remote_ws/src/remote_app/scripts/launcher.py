import subprocess
import rospy
from std_msgs.msg import String


running = {}


def doLaunch(launch_info):
    machine_name = launch_info.data.split(',')[0]
    ip_addr = launch_info.data.split(',')[1]
    proc = subprocess.Popen(
        ['roslaunch', '~/JayBotsDT/launch/remote_standard.launch',
        'machine_name:={}'.format(machine_name), 'ip_addr:={}'.format(ip_addr)])
    running[machine_name] = proc

def killRover(kill_info):
    machine_name = launch_info.data
    running[machine_name].terminate()

def setup_node():

    rospy.init_node('launcher');
    rospy.Subscriber('launch_rover', String, doLaunch)
    rospy.Subscriber('kill_rover', String, killRover)

    rate = rospy.Rate(1)

    while not rospy.is_shutdown():

        for machine_name in running.keys():
            status = running[machine_name].poll()
            if status is not None:
                print(status)
                del running[machine_name]

        rate.sleep()

if __name__ == '__main__':
    setup_node()
