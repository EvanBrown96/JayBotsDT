#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from threading import RLock
from Queue import Queue
from threading import Thread
from avoidance import start_sensors
from driver_node import setup_driver
from gpiozero import LED
from lidar_avoidance import start_lidar
from jaybot.msg import Threshold
import random

BLINKER_GPIO = 25
led = None

driver_queue = None
avoidance_queue = None

class Mode:
    MANUAL = 0
    AUTONOMOUS = 1

mode = Mode.MANUAL
movement_state = 'ss'

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

        # if cmd[2] == 'f':
        #     movement_state = 'forward'
        # elif cmd[2] == 'b':
        #     movement_state = 'backward'
        # elif cmd[3] == 'r':
        #     movement_state = 'right'
        # elif cmd[3] == 'l':
        #     movement_state = 'left'
        # else:
        #     movement_state = 'stop'

        if movement_state[0] == 'f' and stop_fwd_movement:
            driver_queue.put("ss")
        else:
            driver_queue.put(movement_state)

    elif cmd[0] == 'a':

        mode = Mode.AUTONOMOUS
        state = "find_wall"
        autonomousSet()
        led.blink(0.5, 2.5)

    lock.release()

# def xor(a, b):
#     return (a and b) or not (a or b)

def get_next_state():
    
    if state == "find_wall":
        if sensors["fwd_lidar"]:
            return "make_right"
    elif state == "make_right":
        if not sensors["fwd_lidar"]:
            return "forward"
    elif state == "forward":
        if sensors["fwd_lidar"]:
            return "make_right"
        elif sensors["right_lidar"]:
            return "too_close"
        elif not sensors["right_fwd_lidar"]:
            return "too_far"
    elif state == "too_close":
        if sensors["fwd_lidar"]:
            return "make_right"
        elif not sensors["right_lidar"]:
            return "forward"
    elif state == "too_far":
        if sensors["fwd_lidar"]:
            return "make_right"
        elif sensors["right_fwd_lidar"]:
            return "forward"
        elif sensors["right_bck_lidar"]:
            return "find_wall"
    return state

def autonomousSet():
    global state

    while state != get_next_state():
        state = get_next_state()
        rospy.loginfo("state changed to {}".format(state))

    if state in ["find_wall", "forward"]:
        driver_queue.put("fs")
    elif state == "make_right":
        driver_queue.put("sl")
    elif state == "too_close":
        driver_queue.put("fl")
    elif state == "too_far":
        driver_queue.put("fr")

    # if sensors["fwd_lidar"]:
    #     if xor(sensors["left_lidar"], sensors["right_lidar"]):
    #         if xor(sensors["left_us"], sensors["right_us"]):
    #             driver_queue.put(random.choice(['right', 'left']))
    #         elif sensors["left_us"]:
    #             driver_queue.put('right')
    #         elif sensors["right_us"]:
    #             driver_queue.put('left')
    #     elif sensors["left_lidar"]:
    #         driver_queue.put('right')
    #     elif sensors["right_lidar"]:
    #         driver_queue.put('left')
    # else:
    #     if sensors["left_us"] and sensors["right_us"]:
    #         driver_queue.put(random.choice(['right', 'left']))
    #     elif sensors["left_us"]:
    #         driver_queue.put('right')
    #     elif sensors["right_us"]:
    #         driver_queue.put('left')
    #     else:
    #         driver_queue.put('forward')

def thresh_handler(thresh_queue):
    rospy.loginfo("starting thresh_handler")

    while not rospy.is_shutdown():
        thresh = thresh_queue.get()

        sensors[thresh.sensor] = thresh.in_range
        rospy.loginfo("sensor {} in range: {}".format(thresh.sensor, thresh.in_range))

        lock.acquire()

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

def setup_node():
    global driver_queue, avoidance_queue, led

    rospy.init_node('movement_logic')
    rospy.loginfo("starting movement_logic")

    rospy.Subscriber('user_cmd', String, commandCallback)
    rospy.loginfo("subscribed to user_cmd")

    driver_queue = Queue()
    avoidance_queue = Queue()
    Thread(target=setup_driver, args=(driver_queue, )).start()
    Thread(target=start_sensors, args=(avoidance_queue, )).start()
    Thread(target=thresh_handler, args=(avoidance_queue, )).start()
    Thread(target=start_lidar, args=(avoidance_queue, )).start()

    led = LED(BLINKER_GPIO)

    rospy.spin()

    driver_queue.put(None)
    avoidance_queue.put(Threshold("", False))

    rospy.loginfo("stopping movement_logic")


if __name__ == '__main__':
    setup_node()
