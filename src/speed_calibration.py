"""
open-loop calibration.

Sweeps the duty cycle and prints the measured speed of each wheel
in encoder steps per second. 

Output: deadband and feedforward gain for the left and right motor.
"""

from time import sleep, monotonic
from gpiozero import DigitalOutputDevice, Motor

from encoder_client import EncoderClient

slp = DigitalOutputDevice('GPIO17')

left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)

# Encoders are read from the encoderd daemon, NOT gpiozero.
# Do not create RotaryEncoder objects on GPIO25/20/26/5: the daemon owns them.
enc = EncoderClient()

LEFT_SIGN = -1
RIGHT_SIGN = 1


def measure(seconds=1.0) -> tuple[float, float]:
    """Returns the measured speed of the left and right wheels in steps/s."""
    left0, right0 = enc.read()
    time0 = monotonic()
    sleep(seconds)
    left1, right1 = enc.read()
    dt = monotonic() - time0

    # speed = (where we are - where we started) / elapsed time
    return (LEFT_SIGN * (left1 - left0) / dt,
            RIGHT_SIGN * (right1 - right0) / dt)



def fit_line(points, min_speed=20.0):
    """Least-squares fit of duty = deadband + kf * speed.
 
    points: list of (duty, speed).
    Points where the wheel barely moved (stalled below the deadband)
    are skipped, because they don't lie on the line.
    """
    pts = [(s, d) for d, s in points if s > min_speed]  # (speed, duty)
 
    n = len(pts)
    if n < 2:
        raise ValueError("Need at least 2 points where the wheel moved")
 
    mean_s = sum(s for s, _ in pts) / n
    mean_d = sum(d for _, d in pts) / n
 
    cov = sum((s - mean_s) * (d - mean_d) for s, d in pts)
    var = sum((s - mean_s) ** 2 for s, _ in pts)
    if var == 0:
        raise ValueError("All speeds identical, cannot fit a line")
 
    kf = cov / var                      # duty per (step/s)
    deadband = mean_d - kf * mean_s     # duty where the wheel just starts moving
    return kf, deadband, n


def duty_for(speed, kf, deadband):
    """Feedforward: target speed (steps/s, signed) -> Motor.value in [-1, 1]."""
    if speed == 0:
        return 0.0
    duty = min(1.0, deadband + kf * abs(speed))
    return duty if speed > 0 else -duty

left_data, right_data = [], []

try:
    slp.on()        # wake the driver, otherwise the Motor objects do nothing
    sleep(0.01)
 
    print("duty, left_steps_per_s, right_steps_per_s")
    for i in range(1, 11):
        duty = i / 10
        left.value = duty
        right.value = duty
        sleep(0.7)  # let the motors reach a steady speed
        l, r = measure(1.0)
        left_data.append((duty, l))
        right_data.append((duty, r))
        print(f"{duty:.1f}, {l:9.1f}, {r:9.1f}")
finally:
    left.stop()     # stop the motors first, then put the driver to sleep
    right.stop()
    slp.off()
    enc.close()

# The daemon can only process STEPS * RATE / 4 = 2100 steps/s per wheel with
# its default settings. Above that its queue backs up and the counts lag.
DAEMON_LIMIT = 2100
for name, data in (("L", left_data), ("R", right_data)):
    top = max(abs(s) for _, s in data)
    if top > 0.9 * DAEMON_LIMIT:
        print(f"WARNING {name}: top speed {top:.0f} steps/s is near the daemon "
              f"limit of {DAEMON_LIMIT}. Raise STEPS in encoderd.cpp.")
 
 
for name, data in (("L", left_data), ("R", right_data)):
    if all(s <= 0 for _, s in data):
        print(f"{name}: no positive speed measured. Check wiring, slp, or {name}_SIGN.")
        continue
    kf, deadband, n = fit_line(data)
    print(f"{name}_KF, {name}_DEADBAND = {kf:.6f}, {deadband:.3f}  (fit from {n} points)")