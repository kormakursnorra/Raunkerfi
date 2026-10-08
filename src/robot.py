#! /usr/bin/env python3

import os
import time
import curses

from gpiozero import DigitalOutputDevice, Motor, Robot, MCP3008


def read_volts3V3(samples=20):
    return sum(sensor.value for _ in range(samples)) / samples * 3.3

def volts_to_cm(v):
    return 67.7 / (v + 0.144) - 5.6

try:
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(1)

    stdscr.addstr(0, 10, "Hit 'q' to quit")
    stdscr.nodelay(1)  # nodelay(1) give us a -1 back when nothing is pressed
    direction = None

    # Configure your own pins.
    slp = DigitalOutputDevice('GPIO17')
    slp.on()
    robot = Robot(
        left=Motor('GPIO12', 'GPIO18'),
        right=Motor('GPIO19', 'GPIO13')
        )

    sensor = MCP3008(channel=0)  # CE0 by default

    # Here starts the code to make the robot move
    while direction != ord('q'):
        stdscr.refresh()

        speed = robot.value

        volts = read_volts3V3()
        distance = volts_to_cm(volts)
    
        if volts > 2.35:
            print(f"too close, under 20 cm  ({volts:.2f} V)")
        elif volts < 0.38:
            print(f"nothing in range  ({volts:.2f} V)")
        else:
            print(f"{distance:.0f} cm  ({volts:.2f} V)")

        

        # direction = stdscr.getch()  # Gets the key which is pressed
        # if direction == ord('e'):
        #     stdscr.addstr(1, 10, "Stop    ")
        #     robot.stop()
        # if direction == ord('a'):
        #     stdscr.addstr(1, 10, "Left    ")
        #     robot.left()
        # if direction == ord('s'):
        #     stdscr.addstr(1, 10, "Backward")
        #     robot.backward()
        # if direction == ord('d'):
        #     stdscr.addstr(1, 10, "Right   ")
        #     robot.right()
        # if direction == ord('w'):
        #     stdscr.addstr(1, 10, "Forward ")
        #     robot.forward()
        time.sleep(0.04)
finally:
    # Important to set everthing back by end of the script
    slp.off()
    curses.nocbreak()
    stdscr.keypad(0)
    curses.endwin()