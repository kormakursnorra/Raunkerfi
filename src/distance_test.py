from gpiozero import MCP3008
from time import sleep

sensor = MCP3008(channel=0)  # CE0 by default


def read_volts3V3(samples=20):
    return sum(sensor.value for _ in range(samples)) / samples * 3.3

def volts_to_cm(volts):
    return 67.7 / (volts + 0.144) - 5.6


while True:
    # average a few samples to smooth out noise
    volts = read_volts3V3()
    distance = volts_to_cm(volts)

    if volts > 2.35:
        print(f"too close, under 20 cm  ({volts:.2f} V)")
    elif volts < 0.38:
        print(f"nothing in range  ({volts:.2f} V)")
    else:
        print(f"{distance:.0f} cm  ({volts:.2f} V)")
    sleep(0.2)