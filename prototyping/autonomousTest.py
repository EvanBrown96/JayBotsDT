from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor
import time

DIST_MAX = 2.0

ultrasonic0 = DistanceSensor(24, 23, max_distance = DIST_MAX)
ultrasonic1 = DistanceSensor(22, 27, max_distance = DIST_MAX)

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

time.sleep(5)

#motors off
def motorsStop():
    print ("stop")
    left_spd.value = 0
    left_fwd.off()
    left_bck.off()
    right_spd.value = 0
    right_fwd.off()
    right_bck.off()

#moving forward
def motorsFwd():
    print ("Forward")
    left_spd.value = 0.5
    left_fwd.on()
    left_bck.off()
    right_spd.value = 0.5
    right_fwd.on()
    right_bck.off()

#moving backwards
def motorsBck():
    print "Backward"
    left_spd.value = 0.5
    left_fwd.off()
    left_bck.on()
    right_spd.value = 0.5
    right_fwd.off()
    right_bck.on()

#turning left
def motorsLeft():
    print ("Left")
    left_spd.value = 0.5
    left_fwd.off()
    left_bck.on()
    right_spd.value = 0.5
    right_fwd.on()
    right_bck.off()

#turning right
def motorsRight():
    print ("Right")
    left_spd.valeu = 0.5
    left_fwd.on()
    left_bck.off()
    right_spd.value = 0.5
    right_fwd.off()
    right_bck.on()

motorsStop()
count = 0
while True:
    ult0 = 100*ultrasonic0.distance
    ult1 = 100*ultrasonic1.distance
    print("1: {}, 2: {}".format(ult0, ult1))
    time.sleep(0.5)
    flag = 0
    if ult0 < 15 or ult1 < 15:
        count = count + 1
        motorsStop()
        time.sleep(1)
        motorsBck()
        time.sleep(1.5)
        if count%3 == 1 & flag == 0:
            motorsRight()
            flag = 1
        else
            motorsLeft()
            flag = 0
        time.sleep(1.5)
        motorsStop()
        time.sleep(1)
    else:
        motorsFwd()
        flag = 0
