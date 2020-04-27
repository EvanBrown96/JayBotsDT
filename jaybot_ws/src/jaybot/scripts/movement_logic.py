#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from threading import RLock
from Queue import Queue
from threading import Thread
from avoidance import start_sensors
from driver_node import setup_driver
from gpiozero import LED

BLINKER_GPIO = 25
led = None

driver_queue = None
avoidance_queue = None

class Mode:
    MANUAL = 0
    AUTONOMOUS = 1

mode = Mode.MANUAL
movement_state = 'stop'

sensors = {
    "left_us": False,
    "right_us": False,
    "fwd_lidar": False,
    "left_lidar": False,
    "right_lidar": False
}

stop_fwd_movement = False

def commandCallback(user_command):
    global mode, movement_state

    cmd = user_command.data

    if cmd[0] == 'm':
        led.off()
        mode = Mode.MANUAL
        if cmd[2] == 'f':
            movement_state = 'forward'
        elif cmd[2] == 'b':
            movement_state = 'backward'
        elif cmd[3] == 'r':
            movement_state = 'right'
        elif cmd[3] == 'l':
            movement_state = 'left'
        else:
            movement_state = 'stop'

        if movement_state == 'forward' and stop_fwd_movement:
            driver_queue.put("stop")
        else:
            driver_queue.put(movement_state)

    elif cmd[0] == 'a':

        mode = Mode.AUTONOMOUS
        autonomousSet()
        led.blink(0.5, 2.5)


def autonomousSet():
    if sensors["left_us"]:
        driver_queue.put('right')
    elif sensors["right_us"]:
        driver_queue.put('left')
    else:
        driver_queue.put('forward')


def thresh_handler(thresh_queue):
    rospy.loginfo("starting thresh_handler")

    while not rospy.is_shutdown():
        thresh = thresh_queue.get()

        sensors[thresh.sensor] = thresh.in_range
        rospy.loginfo("sensor {} in range: {}".format(thresh.sensor, thresh.in_range))

        was_stopped = stop_fwd_movement
        update_stop_fwd_movement()

        if mode == Mode.MANUAL:
            if movement_state == 'forward' and not was_stopped:
                if stop_fwd_movement:
                    driver_queue.put("stop")
                else:
                    driver_queue.put(movement_state)

        elif mode == Mode.AUTONOMOUS:
            autonomousSet()

    rospy.loginfo("stopping thresh_handler")

def update_stop_fwd_movement():
    global stop_fwd_movement
    stop_fwd_movement = sensors["left_us"] or sensors["right_us"] or sensors["fwd_lidar"]

# def leftAvoidance(left_status):
#     global left_avoiding

#     left_avoiding = left_status.data

#     if mode == Mode.MANUAL:
#         if movement_state == 'forward' and not right_avoiding:
#             if left_avoiding:
#                 vel_cmd_pub.publish('stop')
#             else:
#                 vel_cmd_pub.publish(movement_state)

#     elif mode == Mode.AUTONOMOUS:
#         autonomousSet()

#     callback_lock.release()


# def rightAvoidance(right_status):
#     global right_avoiding

#     callback_lock.acquire()

#     right_avoiding = right_status.data

#     if mode == Mode.MANUAL:
#         if movement_state == 'forward' and not left_avoiding:
#             if right_avoiding:
#                 vel_cmd_pub.publish('stop')
#             else:
#                 vel_cmd_pub.publish(movement_state)

#     elif mode == Mode.AUTONOMOUS:
#         autonomousSet()

#     callback_lock.release()

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

    led = LED(BLINKER_GPIO)

    rospy.spin()

    driver_queue.put(None)

    rospy.loginfo("stopping movement_logic")


if __name__ == '__main__':
    setup_node()
