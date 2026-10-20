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

    # control effort = (where we want to be - where we are) * 1/elapsed time
    return (left_enc.steps - left0) / dt, (right_enc.steps - right0) / dt

try:
    print("duty, left_steps_per_s, right_steps_per_s")
    for i in range(1, 11):
        duty = i / 10
        left.value = duty
        right.value = duty
        sleep(0.7)  # letting the motor reach a steady speed
        l, r = measure(1.0)
        print(f"{duty:.1f}, {l:9.1f}, {r:9.1f}")
finally:
    left.stop()
    right.stop()