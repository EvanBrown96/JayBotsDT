from gpiozero import DigitalOutputDevice

trigger = DigitalOutputDevice(17)
trigger.blink(on_time=0.00001, off_time=1)