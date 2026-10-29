#! /usr/bin/env python3

import os
import board

import adafruit_icm20x
import adafruit_tcs34725

from time import sleep, monotonic
from gpiozero import MCP3008

from .wheel_control import Driver, LOOP_DT

MIN_SPEED = 375    # constant for line following.

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
        self.driver       = Driver()
        self.color_sensor = adafruit_tcs34725.TCS34725( i2c )
        self.imu_compass  = adafruit_icm20x.ICM20948( i2c, address=0x69 )
        self.infra_sensor = MCP3008( channel=0 )  # CE0 by default

        # Additional configurations
        self.color_sensor.integration_time = 150  # ms, 2.4 to 614.4; longer = more sensitive
        self.color_sensor.gain = 4                # 1, 4, 16, or 60

    

    def follow_line(self):
        """Main loop of the robot.  """
        try:
            self.driver.start()   # wake the driver only once everything is set up
            sleep(0.01)
            
            t_start  = monotonic()
            t_last   = t_start
            nxt_tick = t_start

            while True:
                nxt_tick += LOOP_DT
                # Wait first, so dt is never zero on the first pass
                sleep(max(0.0, nxt_tick - monotonic()))

                t_now  = monotonic()
                dtime  = t_now - t_last
                t_last = t_now
                time   = t_now - t_start

                target = target_at(time)
                if target is None:
                    break

                self.driver.set_targets(target, target, dtime)
                ld, rd = self.driver.update(dtime)

                l, r = self.driver.left, self.driver.right
            
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
        finally:
            self.shutdown()

    def shutdown(self):
        self.driver.stop()


def main():
    marvin = Marvin()
    marvin.follow_line()



if __name__ == "__main__":
    main()