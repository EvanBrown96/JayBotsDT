#!/usr/bin/python
import RPi.GPIO as GPIO
import time

class Sonar():

    def __init__(self, gpio_trigger, gpio_echo, range_min=10, range_max=400):

        GPIO.setmode(GPIO.BCM)

        self._gpio_trigger  = gpio_trigger
        self._gpio_echo     = gpio_echo
        self._range_min     = range_min
        self._range_max     = range_max
        self._is_reading    = False

        self._speed_sound   = 17150.0

        self._last_time_reading = 0
        self._timeout       = range_max/self._speed_sound*2

        GPIO.setup(gpio_trigger, GPIO.OUT)
        GPIO.setup(gpio_echo, GPIO.IN)

        GPIO.output(gpio_trigger, GPIO.LOW)
        time.sleep(1)


    def get_range(self):
        self._is_reading = True

        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)

        GPIO.output(self._gpio_trigger, GPIO.HIGH)
        time.sleep(0.00001)
        GPIO.output(self._gpio_trigger, GPIO.LOW)


        pulse_start_time = time.time()
        pulse_end_time = time.time()

        while GPIO.input(self._gpio_echo)==0:
            pulse_start_time = time.time()

        time0= time.time()
        while GPIO.input(self._gpio_echo)==1:
            pulse_end_time = time.time()

        self._last_time_reading = time.time()
        self._is_reading = False

        pulse_duration = pulse_end_time - pulse_start_time
        distance = pulse_duration * self._speed_sound

        if distance > self._range_max:
            distance = self._range_max

        if distance < self._range_min:
            distance = self._range_min

        return(distance)

    @property
    def is_reading(self):
        return(self._is_reading)

if __name__ == "__main__":

    PIN_TRIGGER = 23
    PIN_ECHO = 24

    PIN_TRIGGER = 27
    PIN_ECHO = 22

    sonar = Sonar(PIN_TRIGGER, PIN_ECHO)

    while True:
        d = sonar.get_range()
        if d>0: print "Distance = %4.1f cm"%d
