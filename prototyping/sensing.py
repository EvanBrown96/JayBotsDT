from gpiozero import DistanceSensor
import time

test = DistanceSensor(24, 23, max_distance=2.0)

while True:
    print(test.distance)
    time.sleep(0.5)