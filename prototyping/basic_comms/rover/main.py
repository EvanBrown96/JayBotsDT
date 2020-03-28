import socket
from gpiozero import DigitalOutputDevice, PWMOutputDevice, DistanceSensor
import sys
import time
import threading

PORT = 10001
sock = socket.socket()
sock.bind(('', PORT))
sock.listen()

sock2 = socket.socket()
sock2.bind(('', PORT+1))
sock2.listen()

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

DIST_MAX = 2.0

test = DistanceSensor(24, 23, max_distance=DIST_MAX)
test2 = DistanceSensor(22, 27, max_distance=DIST_MAX)

spd_mappings = {
    "m-ss": (0, 0),
    "m-fs": (1, 1),
    "m-bs": (-1, -1),
    "m-sr": (1, -1),
    "m-fr": (1, 0.5),
    "m-br": (-1, -0.5),
    "m-sl": (-1, 1),
    "m-fl": (0.5, 1),
    "m-bl": (-0.5, 1)
}

def one():
    while True:
        try:
            conn, addr = sock.accept()

            while True:
                data = conn.recv(1024)

                if not data:
                    break
                if data != "persist":

                    if data in spd_mappings.keys():
                        left_velocity = spd_mappings[data][0]
                        right_velocity = spd_mappings[data][1]

                        left_spd.value = 0.5*abs(left_velocity)
                        right_spd.value = 0.5*abs(right_velocity)
                    else:
                        x = int(float(data.split(',')[0]))
                        y = int(float(data.split(',')[1]))

                        left_velocity = (y+x)/2
                        right_velocity = (y-x)/2

                        left_spd.value = float(abs(left_velocity))/100
                        right_spd.value = float(abs(right_velocity))/100

                    if left_velocity > 0:
                        left_fwd.on()
                        left_bck.off()
                    elif left_velocity < 0:
                        left_fwd.off()
                        left_bck.on()
                    else:
                        left_fwd.off()
                        left_bck.off()

                    if right_velocity > 0:
                        right_fwd.on()
                        right_bck.off()
                    elif right_velocity < 0:
                        right_fwd.off()
                        right_bck.on()
                    else:
                        right_fwd.off()
                        right_bck.off()

        except KeyboardInterrupt:
            conn.close()
            sys.exit(0)
        except:
            pass

def two():
    last_time = 0

    while True:
        try:
            conn, addr = sock2.accept()

            while True:
                if time.time() < last_time or time.time() > last_time + 0.5:
                    msg = "L {},R {}".format(100 * test.distance, 100 * test2.distance)
                    #msg = "L {},R {}".format(100, 101)
                    conn.send(msg.encode('utf-8'))
                    last_time = time.time()
        except KeyboardInterrupt:
            conn.close()
            sys.exit(0)
        except:
            pass

threading.Thread(None, one).start()
threading.Thread(None, two).start()

