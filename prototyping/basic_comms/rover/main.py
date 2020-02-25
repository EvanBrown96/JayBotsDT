import socket
from gpiozero import DigitalOutputDevice, PWMOutputDevice

PORT = 10001
sock = socket.socket()
sock.bind(('', PORT))
sock.listen()

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=800)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=800)

while True:
    conn, addr = sock.accept()

    while True:
        data = conn.recv(1024).decode('utf-8')
        if not data:
            break
        if data != "persist":
            x = int(float(data.split(',')[0]))
            y = int(float(data.split(',')[1]))

            left_velocity = (y+x)/2
            right_velocity = (y-x)/2

            left_spd.value = float(abs(left_velocity))/200
            right_spd.value = float(abs(right_velocity))/200

            if left_velocity > 0:
                left_fwd.on()
                left_bck.off()
            elif left_velocity < 0:
                left_fwd.off()
                left_bck.on()
            else:
                left_fwd.off()
                left_fwd.on()

            if right_velocity > 0:
                right_fwd.on()
                right_bck.off()
            elif right_velocity < 0:
                right_fwd.off()
                right_bck.on()
            else:
                right_fwd.off()
                right_bck.on()
