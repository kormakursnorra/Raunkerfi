"""
closed-loop speed control test on the floor.

Runs a step profile forward, then the same profile backward, so the robot
ends up roughly where it started. Give it 2-3 m of room. Logs every tick to
pid_log.csv.

The controller itself lives in marvin/wheel_control.py (feedforward + PID,
anti-windup, kick-start).
"""

import csv
from time import sleep, monotonic

from rover.wheel_control import DriveController, LOOP_DT

KP = 0.00003   # reaction to the current error
KI = 0.0001    # removes the leftover steady error
KD = 0.0       # reaction to the rate of change of the error

# (time_s, target steps/s). None ends the test.
# Forward steps, stop, then the same steps backward.
PROFILE = [(0.0, 600), (3.0, 1200), (6.0, 800), (9.0, 0),
           (10.0, -600), (13.0, -1200), (16.0, -800), (19.0, 0),
           (20.0, None)]


def target_at(t):
    current = 0
    for start, value in PROFILE:
        if t >= start:
            current = value
    return current


def main():
    drive = DriveController(k_values={"kp": KP, "ki": KI, "kd": KD})
    try:
        drive.start()   # wake the driver only once everything is set up
        sleep(0.01)
        with open("pid_log.csv", "w", newline="", encoding="utf-8") as f:
            log = csv.writer(f)
            log.writerow(["time", "target",
                          "left_target", "left_raw", "left_speed", "left_duty",
                          "right_target", "right_raw", "right_speed", "right_duty"])

            t_start = monotonic()
            t_last = t_start
            next_tick = t_start

            while True:
                next_tick += LOOP_DT
                # Wait first, so dt is never zero on the first pass
                sleep(max(0.0, next_tick - monotonic()))

                t_now = monotonic()
                dt = t_now - t_last
                t_last = t_now
                t = t_now - t_start

                target = target_at(t)
                if target is None:
                    break

                drive.set_targets(target, target, dt)
                ld, rd = drive.update(dt)

                l, r = drive.left, drive.right
                log.writerow([f"{t:.3f}", target,
                              f"{l.target:.1f}", f"{l.raw_speed:.1f}",
                              f"{l.speed:.1f}", f"{ld:.3f}",
                              f"{r.target:.1f}", f"{r.raw_speed:.1f}",
                              f"{r.speed:.1f}", f"{rd:.3f}"])
    finally:
        drive.shutdown()   # stops the motors, then puts the driver to sleep


if __name__ == "__main__":
    main()