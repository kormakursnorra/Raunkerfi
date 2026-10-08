#! /usr/bin/env python3

import os
import time
import board
import curses

import adafruit_icm20x
import adafruit_tcs34725

from gpiozero import DigitalOutputDevice, Motor, Robot, MCP3008, RotaryEncoder


def read_volts3v3(sensor: MCP3008, samples=20):
    """Takes the mean of voltage samples."""
    return sum(sensor.value for _ in range(samples)) / samples * 3.3

def volts_to_cm(v):
    """Converts voltage to centimeters."""
    return 67.7 / (v + 0.144) - 5.6

class Marvin:
    """Class representing the rover robot."""
    def __init__(self):
        i2c = board.I2C()
        # Initialize all hardware components
        self.color_sensor = adafruit_tcs34725.TCS34725(i2c)
        self.imu_compass = adafruit_icm20x.ICM20948(i2c, address=0x69)
        self.infra_sensor = MCP3008(channel=0)  # CE0 by default

        # Configure the motor pins.
        self.slp = DigitalOutputDevice('GPIO17')
        self.wheels = Robot(
            left=Motor('GPIO12', 'GPIO18'),
            right=Motor('GPIO19', 'GPIO13')
            )

        # Additional configurations
        self.slp.on()
        self.color_sensor.integration_time = 150  # ms, 2.4 to 614.4; longer = more sensitive
        self.color_sensor.gain = 4                # 1, 4, 16, or 60
        
        # Also initialize interactive screen
        self.stdscr = curses.initscr()
        self.command = None
        
        curses.cbreak()
        self.stdscr.keypad(1)
        self.stdscr.addstr(0, 10, "Hit 'q' to quit")
        self.stdscr.nodelay(1)

    def follow_line(self):
        try:
            self.stdscr.refresh()
            
            speed = self.wheels.value

            # Get the current distance between rover -> object infront
            volts = read_volts3v3(self.infra_sensor)
            distance = volts_to_cm(volts)

            # Get the current color values
            r, g, b, clear = self.color_sensor.color_raw

            # Ge the current spacial position of rover
            ax, ay, az = self.imu_compass.acceleration   # m/s^2
            gx, gy, gz = self.imu_compass.gyro           # rad/s
            mx, my, mz = self.imu_compass.magnetic       # microtesla

            time.sleep(0.04)

        except KeyboardInterrupt:
            if self.command == ord('q' or 'Q'):
                self.stop()

    def stop(self):

        self.wheels.stop()

        self.slp.off()
        curses.nocbreak()
        self.stdscr.keypad(0)
        curses.endwin()


def main():
    marvin = Marvin()
    try:
        while True:
            marvin.follow_line()
    finally:
        marvin.stop()



if __name__ == "__main__":
    main()