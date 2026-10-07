from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)  # CE0 by default

while True:
    print(f"{sensor.value * 3.3:.2f} V")
    sleep(0.2)