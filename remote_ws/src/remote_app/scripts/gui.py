import subprocess
from subprocess import Popen
import rospy
import sys
import os.path


if len(sys.argv) < 2:
    print("Operation: gui.py [launch_dir]")
    sys.exit(1)


GUI_LOCATION = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(sys.argv[1]))))),
    "RoverController", "RoverController", "bin", "Release", "RoverController.exe")


def start_gui():
    rospy.init_node('gui')

    rate = rospy.Rate(1)

    print(GUI_LOCATION)
    gui_proc = Popen([GUI_LOCATION])

    while not rospy.is_shutdown():
        if gui_proc.poll() is not None:
            rospy.signal_shutdown('gui closed')
        rate.sleep()

    if gui_proc.poll() is None:
        gui_proc.terminate()


if __name__ == "__main__":
    start_gui()
