"""
open-loop calibration.

Sweeps the duty cycle and prints the measured speed of each wheel
in encoder steps per second. 

Output: deadband and feedforward gain for the left and right motor.
"""

from time import sleep, monotonic
from gpiozero import Motor, RotaryEncoder

left = Motor('GPIO12', 'GPIO18', pwm=True)
right = Motor('GPIO19', 'GPIO13', pwm=True)

left_enc = RotaryEncoder('GPIO25', 'GPIO20', max_steps=0)
right_enc = RotaryEncoder('GPIO26', 'GPIO5', max_steps=0)

def measure(seconds=1.0) -> tuple[float, float]:
    """Returns a measurement of the left and right motors efforts"""
    left0, right0 = left_enc.steps, right_enc.steps # where we are (left- and right motor)
    time0 = monotonic()
    sleep(seconds)
    dt = monotonic() - time0 # derived/elapsed time

    # speed = (where we are - where we started) * elapsed time
    return (left_enc.steps - left0) / dt, (right_enc.steps - right0) / dt


def fit_line(points, mind_speed=20.0):
    """Least-squares fit of duty = deadband + kf * speed.
 
    Points where the wheel barely moved (stalled below the deadband)
    are skipped, because they don't lie on the line.
    """
    pts = [(s, d) for d, s in points if s > mind_speed]

    n = len(pts) # number of measurements
    if n < 2:
        raise ValueError("Need at least 2 points where the wheel moved")
    mean_d = sum(d for d, _ in pts) / n # get the mean of summed duty cycles
    mean_s = sum(s for _, s in pts) / n # get the mean of summed steps_per_s

    # covariance: for each point check if speed and duty are above or below their means
    num = sum((d - mean_d) * (s - mean_s) for d, s in pts)

    # variance: how spread out are the speeds
    den = sum((d - mean_d) ** 2 for d, _ in pts)

    slope = num / den                   # step/s per unit duty
    intercept = mean_d - slope * mean_s # speed = intercept + slope * duty
    
    # feedforward gain (kf) is what converts desired speed to the duty cycle that produces it.
    kf = 1.0 / slope

    # deadband is the duty where the motor barely starts turning.
    deadband = -intercept / slope
    return kf, deadband, n


left_data, right_data = [], []

try:
    print("duty, left_steps_per_s, right_steps_per_s")
    for i in range(1, 11):
        duty = i / 10
        left.value = duty
        right.value = duty
        sleep(0.7)  # letting the motor reach a steady speed
        l, r = measure(1.0)
        left_data.append((duty, l))
        right_data.append((duty, r))
        print(f"{duty:.1f}, {l:9.1f}, {r:9.1f}")
finally:
    left.stop()
    right.stop()


for name, data in (("L", left_data), ("R", right_data)):
    if all(s <= 0 for _, s in data):
        print(f"{name}: no positive speed measured.")
        continue
    kf, deadband, n = fit_line(data)
    print(f"{name}:_KF, {name}_DEADBAND = {kf:.6f}, {deadband:.3f}  (fit from {n} points)")