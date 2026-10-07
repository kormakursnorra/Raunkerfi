from gpiozero import MCP3008
from time import sleep

def voltage_to_cm(voltage):
    if voltage <= 0:
        return None
    return 60.374 * (voltage ** -1.16)

sensor = MCP3008(channel=0)  # CE0 by default
channels = [MCP3008(channel=i) for i in range(8)]

#while True:
    #print("  ".join(f"CH{i}:{c.value * 3.3:.2f}" for i, c in enumerate(channels)))
    #sleep(0.3)

while True:
    # average a few samples to smooth out noise
    volts = sum(sensor.value for _ in range(10)) / 10 * 3.3
    if 0.4 <= volts <= 2.7:
        print(f"{65 * volts ** -1.10:.0f} cm  ({volts:.2f} V)")
    else:
        print(f"out of range  ({volts:.2f} V)")
    sleep(0.2)

#while True:
 #   print(f"{sensor.value * 3.3:.2f} V")
    #print(f"{voltage_to_cm(sensor.value)} cm")
  #  sleep(0.2)
