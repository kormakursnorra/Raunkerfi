"""
open-loop calibration.

Sweeps the duty cycle and prints the measured speed of each wheel
in encoder steps per second. 

Output: deadband and feedforward gain for the left and right motor.
"""

from time import sleep, monotonic
from gpiozero import DigitalOutputDevice, Motor, RotaryEncoder

slp = DigitalOutputDevice('GPIO17')

left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)

left_enc = RotaryEncoder('GPIO25', 'GPIO20', max_steps=0)
right_enc = RotaryEncoder('GPIO26', 'GPIO5', max_steps=0)

LEFT_SIGN = 1
RIGHT_SIGN = -1


def measure(seconds=1.0) -> tuple[float, float]:
    """Returns a measurement of the left and right motors efforts"""
    left0, right0 = left_enc.steps, right_enc.steps
    time0 = monotonic()
    sleep(seconds)
    dt = monotonic() - time0
 
    # speed = (where we are - where we started) / elapsed time
    return (LEFT_SIGN * (left_enc.steps - left0) / dt,
            RIGHT_SIGN * (right_enc.steps - right0) / dt)



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
 
 
for name, data in (("L", left_data), ("R", right_data)):
    if all(s <= 0 for _, s in data):
        print(f"{name}: no positive speed measured. Check wiring, slp, or {name}_SIGN.")
        continue
    kf, deadband, n = fit_line(data)
    print(f"{name}_KF, {name}_DEADBAND = {kf:.6f}, {deadband:.3f}  (fit from {n} points)")
