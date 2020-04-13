from gpiozero import DigitalOutputDevice, PWMOutputDevice
import curses

left_fwd = DigitalOutputDevice(20)
left_bck = DigitalOutputDevice(21)
left_spd = PWMOutputDevice(12, frequency=500)

right_fwd = DigitalOutputDevice(5)
right_bck = DigitalOutputDevice(6)
right_spd = PWMOutputDevice(13, frequency=500)

curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_UP:
            print "Forward"
            left_spd.value = 0.505
            left_fwd.on()
            left_bck.off()
            right_spd.value = 0.5
            right_fwd.on()
            right_bck.off()
        elif char == curses.KEY_DOWN:
            print "Backward"
            left_spd.value = 0.5
            left_fwd.off()
            left_bck.on()
            right_spd.value = 0.5
            right_fwd.off()
            right_bck.on()
        elif char == curses.KEY_LEFT:
            print "Left"
            left_spd.value = 0.5
            left_fwd.off()
            left_bck.on()
            right_spd.value = 0.5
            right_fwd.on()
            right_bck.off()
        elif char == curses.KEY_RIGHT:
            print "Right"
            left_spd.valeu = 0.5
            left_fwd.on()
            left_bck.off()
            right_spd.value = 0.5
            right_fwd.off()
            right_bck.on()
        elif char == 10:
            print "stop"
            left_spd.value = 0
            left_fwd.off()
            left_bck.off()
            right_spd.value = 0
            right_fwd.off()
            right_bck.off()
finally:
    curses.nocbreak(); screen.keypad(0); curses.echo()
    curses.endwin()
