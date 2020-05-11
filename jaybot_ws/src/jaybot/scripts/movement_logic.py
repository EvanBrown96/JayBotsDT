#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from threading import RLock
from Queue import Queue
from threading import Thread
from avoidance import start_sensors
from driver_node import driver_setup
from gpiozero import LED
from lidar_avoidance import start_lidar
from jaybot.msg import Threshold
from std_srvs.srv import SetBool, SetBoolResponse
import random
from path_follow import setup_path_follow, follow_path, cancel_path

BLINKER_GPIO = 25
led = None

driver_queue = None
# path_queue = None
avoidance_queue = None

class Mode:
    MANUAL = 0
    AUTONOMOUS = 1
    PATHFINDING = 2

class AutoState:
    FIND_WALL = 0
    MAKE_RIGHT = 1
    FORWARD = 2
    TOO_CLOSE = 3
    TOO_FAR = 4

mode = Mode.MANUAL
movement_state = 'ss'
manual_avoidance = True

lock = RLock()

sensors = {
    "left_us": False,
    "right_us": False,
    "fwd_lidar": False,
    "left_lidar": False,
    "right_lidar": False,
    "right_fwd_lidar": False,
    "right_bck_lidar": False
}

stop_fwd_movement = False
state = None

def commandCallback(user_command):
    global mode, movement_state, state

    cmd = user_command.data

    lock.acquire()

    if cmd[0] == 'm':
        led.off()
        mode = Mode.MANUAL
        movement_state = cmd[2:4]

        if movement_state[0] == 'f' and stop_fwd_movement and manual_avoidance:
            driver_queue.put("ss")
        else:
            driver_queue.put(movement_state)

    elif cmd[0] == 'a':

        mode = Mode.AUTONOMOUS
        state = AutoState.FIND_WALL
        autonomousSet()
        led.blink(0.5, 2.5)

    elif cmd[0] == 'p':
        mode = Mode.PATHFINDING
        lock.release()
        follow_path()
        lock.acquire()

    cancel_path()
    lock.release()


def get_next_state():

    if state == AutoState.FIND_WALL:
        if stop_fwd_movement:
            return AutoState.MAKE_RIGHT

    elif state == AutoState.MAKE_RIGHT:
        if not stop_fwd_movement:
            return AutoState.FORWARD

    elif state == AutoState.FORWARD:
        if stop_fwd_movement:
            return AutoState.MAKE_RIGHT
        elif sensors["right_lidar"]:
            return AutoState.TOO_CLOSE
        elif not sensors["right_fwd_lidar"]:
            return AutoState.TOO_FAR

    elif state == AutoState.TOO_CLOSE:
        if stop_fwd_movement:
            return AutoState.MAKE_RIGHT
        elif not sensors["right_lidar"]:
            return AutoState.FORWARD

    elif state == AutoState.TOO_FAR:
        if stop_fwd_movement:
            return AutoState.MAKE_RIGHT
        elif sensors["right_fwd_lidar"]:
            return AutoState.FORWARD
        elif not sensors["right_bck_lidar"]:
            return AutoState.FIND_WALL

    return state


def autonomousSet():
    global state

    while state != get_next_state():
        state = get_next_state()
        rospy.loginfo("state changed to {}".format(state))

    if state in [AutoState.FIND_WALL, AutoState.FORWARD]:
        driver_queue.put("fs")
    elif state == AutoState.MAKE_RIGHT:
        driver_queue.put("sl")
    elif state == AutoState.TOO_CLOSE:
        driver_queue.put("fl")
    elif state == AutoState.TOO_FAR:
        driver_queue.put("fr")


def thresh_handler(thresh_queue):
    rospy.loginfo("starting thresh_handler")

    while not rospy.is_shutdown():
        thresh = thresh_queue.get()

        lock.acquire()

        sensors[thresh.sensor] = thresh.in_range
        rospy.loginfo("sensor {} in range: {}".format(thresh.sensor, thresh.in_range))

        was_stopped = stop_fwd_movement
        update_stop_fwd_movement()

        if mode == Mode.MANUAL:
            if movement_state[0] == 'f' and not was_stopped:
                if stop_fwd_movement:
                    driver_queue.put("ss")
                else:
                    driver_queue.put(movement_state)

        elif mode == Mode.AUTONOMOUS:
            autonomousSet()

        lock.release()

    rospy.loginfo("stopping thresh_handler")


def update_stop_fwd_movement():
    global stop_fwd_movement
    stop_fwd_movement = sensors["left_us"] or sensors["right_us"] or sensors["fwd_lidar"]


def set_avoidance(setbool):
    global manual_avoidance
    manual_avoidance = setbool.data
    return SetBoolResponse(True, "")

# def forward_path():
#     while not rospy.is_shutdown():
#         cmd = path_queue.get()
#         if mode == Mode.PATHFINDING:
#             driver_queue.put(cmd)

def setup_node():
    global driver_queue, avoidance_queue, path_queue, led

    rospy.loginfo("starting movement_logic")

    rospy.init_node('movement_logic')
    rospy.loginfo("started node")

    rospy.Subscriber('user_cmd', String, commandCallback)
    rospy.loginfo("subscribed to user_cmd")

    rospy.Service('set_avoidance', SetBool, set_avoidance)
    rospy.loginfo("started service set_avoidance")

    driver_queue = Queue()
    # path_queue = Queue()
    avoidance_queue = Queue()
    Thread(target=driver_setup, args=(driver_queue, )).start()
    Thread(target=start_sensors, args=(avoidance_queue, )).start()
    Thread(target=thresh_handler, args=(avoidance_queue, )).start()
    Thread(target=start_lidar, args=(avoidance_queue, )).start()
    Thread(target=setup_path_follow, args=(driver_queue, )).start()
    # Thread(target=forward_path).start()

    led = LED(BLINKER_GPIO)

    rospy.spin()

    driver_queue.put(None)
    path_queue.put(None)
    avoidance_queue.put(Threshold("", False))

    rospy.loginfo("stopping movement_logic")


if __name__ == '__main__':
    setup_node()
