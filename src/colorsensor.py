#import sys
import time
import board
#import smbus3
import adafruit_tcs34725

i2c = board.I2C()
sensor = adafruit_tcs34725.TCS34725(i2c)

sensor.integration_time = 150  # ms, 2.4 to 614.4; longer = more sensitive
sensor.gain = 4                # 1, 4, 16, or 60

while True:
    r, g, b, clear = sensor.color_raw
    print(f"raw: R={r} G={g} B={b} C={clear}")
    print(f"rgb: {sensor.color_rgb_bytes}  "
          f"temp: {sensor.color_temperature} K  lux: {sensor.lux}")
    time.sleep(1)

# Initialize I2C bus
# bus = smbus3.SMBus(1)
# TCS34725_ADDRESS = 0x29
