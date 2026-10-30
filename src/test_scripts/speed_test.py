from time import sleep
from gpiozero import DigitalOutputDevice, Motor
from src.rover.encoder_client import EncoderClient

slp = DigitalOutputDevice('GPIO17')
left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)
enc = EncoderClient()

try:
    slp.on(); sleep(0.01)
    l0, r0 = enc.read()
    left.value = right.value = 0.1
    sleep(1.0)
    l1, r1 = enc.read()
finally:
    left.stop(); right.stop(); slp.off()

print("left delta:", l1 - l0, " right delta:", r1 - r0)