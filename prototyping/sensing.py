from gpiozero import DistanceSensor
import time

DIST_MAX = 2.0

test = DistanceSensor(24, 23, max_distance=DIST_MAX)
test2 = DistanceSensor(22, 27, max_distance=DIST_MAX)

while True:
    print("1: {}, 2: {}".format(100*test.distance, 100*test2.distance))
    time.sleep(0.5)