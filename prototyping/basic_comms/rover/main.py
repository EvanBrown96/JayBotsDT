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
            velocity = int(data.split(',')[0])
            angle = int(data.split(',')[1])

            if velocity > 0:
                left_fwd.on()
                right_fwd.on()
                left_bck.off()
                right_bck.off()
                left_spd.value = float(velocity)/100
                right_spd.value = float(velocity)/100
            elif velocity < 0:
                left_fwd.off()
                right_fwd.off()
                left_bck.on()
                right_bck.on()
                left_spd.value = float(velocity)/100
                right_spd.value = float(velocity)/100




