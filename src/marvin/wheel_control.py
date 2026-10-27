"""
Shared drive code: feedforward + PID speed control, one controller per wheel.

Control law per wheel:
    duty = sign(target) * deadband + kf * target + PID(target - measured)

Importing this module has no side effects. Hardware is only touched when
you create a Drive. Used by speed_control.py, min_speed_test.py and marvin.py.

Speeds are in encoder steps per second, same units as speed_calibration.py.
"""

import math

from simple_pid import PID
from gpiozero import DigitalOutputDevice, Motor

from .encoder_client import EncoderClient


# encoderd publishes counts every 1/RATE s (RATE = 200 -> 5 ms). Keep
# LOOP_DT well above that so each tick spans many batches.
LOOP_DT = 0.05     # 20 Hz control loop
MAX_ACCEL = 2000   # steps/s^2, ramp limit on the target

# Positive = forward.
LEFT_SIGN = -1
RIGHT_SIGN = 1

# Calibration (from speed_calibration.py) and tuned gains (from speed_control.py).
L_KF, L_DEADBAND = 0.000093, 0.033
R_KF, R_DEADBAND = 0.000095, 0.036
K_VALUES = {"kp": 0.00003, "ki": 0.0001, "kd": 0.0}

# Anti-windup: the integral is frozen only while
#   - the target is still ramping, or
#   - the wheel is more than COAST_BAND steps/s above target AND still slowing
#     down (coasting after a speed drop).
# In every other case it integrates, so it can always fix a wrong feedforward.
COAST_BAND = 50.0      # steps/s

# Kick-start: below KICK_BELOW steps/s with a non-zero target, use at least
# KICK_DUTY so the wheel breaks free of stiction. Your logs show break-away
# at about 0.08-0.09 duty. Set KICK_DUTY = 0.0 to disable.
KICK_DUTY = 0.10
KICK_BELOW = 30.0      # steps/s

class WheelController:
    """Feedforward + PID speed controller for a single wheel."""

    def __init__(self, wheel_args: dict, k_values: dict, start_steps: int,
                 filter_alpha=0.5, max_accel=MAX_ACCEL):
        self.motor: Motor   = wheel_args["motor"]
        self.sign: int      = wheel_args["sign"]
        self.kf: float      = wheel_args["kf"]
        self.deadband: float = wheel_args["deadband"]

        self.alpha = filter_alpha   # 1.0 = no filtering
        self.max_accel = max_accel

        # sample_time=None: we decide when the PID runs.
        # output_limits caps the PID's contribution to +-0.1 duty.
        self.pid = PID(k_values["kp"], k_values["ki"], k_values["kd"],
                       setpoint=0.0, sample_time=None,
                       output_limits=(-0.1, 0.1))

        self.target = 0.0
        self.ramping = False
        self.speed = 0.0       # filtered
        self.raw_speed = 0.0   # unfiltered, last tick
        self.last_steps = self.sign * start_steps

    def reset(self, raw_steps):
        """Stop the motor and forget all history (speed, ramp, integral)."""
        self.motor.stop()
        self.pid.reset()
        self.target = 0.0
        self.ramping = False
        self.speed = 0.0
        self.raw_speed = 0.0
        self.last_steps = self.sign * raw_steps

    def set_target(self, steps_per_s, dt):
        """Moves the target toward steps_per_s, limited by max_accel."""
        step = self.max_accel * dt
        gap = steps_per_s - self.target
        if abs(gap) <= step:
            self.target = float(steps_per_s)   # snap: no float leftovers
            self.ramping = False
        else:
            self.target += math.copysign(step, gap)
            self.ramping = True
        self.pid.setpoint = self.target

    def update(self, raw_steps, dt):
        """Updates the motor from this wheel's encoder count. Returns duty."""
        steps = self.sign * raw_steps
        self.raw_speed = (steps - self.last_steps) / dt
        self.last_steps = steps

        prev_speed = self.speed
        self.speed = self.alpha * self.raw_speed + (1 - self.alpha) * self.speed

        if self.target == 0:
            self.pid.reset()
            self.motor.stop()
            return 0.0

        feedforward = self.kf * self.target + math.copysign(self.deadband, self.target)
        
        # Anti-windup (see COAST_BAND). Everything is measured in the target's
        # direction, so it works the same forwards and backwards.
        # simple_pid accumulates Ki * error * dt into its integral, so zeroing
        # Ki for one call freezes the integral without changing it.
        d = math.copysign(1.0, self.target)
        above = (self.speed - self.target) * d > COAST_BAND
        slowing = (self.speed - prev_speed) * d < 0
        ki = self.pid.Ki
        if self.ramping or (above and slowing):
            self.pid.Ki = 0.0
        try:
            correction = self.pid(self.speed, dt=dt)
        finally:
            self.pid.Ki = ki
 
        duty = feedforward + correction
 
        # Kick-start from (near) standstill, in the target's direction.
        if abs(self.speed) < KICK_BELOW and abs(duty) < KICK_DUTY:
            duty = math.copysign(KICK_DUTY, self.target)
 
        duty = max(-1.0, min(1.0, duty))
 
        self.motor.value = duty
        return duty



class Drive:
    """Both wheels, the driver's sleep pin and the encoder client together."""

    def __init__(self, k_values=K_VALUES):
        self.slp = DigitalOutputDevice('GPIO17')
        left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
        right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)
        self.enc = EncoderClient()

        l0, r0 = self.enc.read()
        self.left = WheelController(
            {"motor": left, "sign": LEFT_SIGN, "kf": L_KF, "deadband": L_DEADBAND},
            k_values, l0)
        self.right = WheelController(
            {"motor": right, "sign": RIGHT_SIGN, "kf": R_KF, "deadband": R_DEADBAND},
            k_values, r0)

    def start(self):
        """Wake the motor driver. Call once everything else is set up."""
        self.slp.on()

    def set_targets(self, left, right, dt):
        self.left.set_target(left, dt)
        self.right.set_target(right, dt)

    def update(self, dt):
        """Reads both encoders in ONE request, updates both wheels.
        Returns (left_duty, right_duty)."""
        l_steps, r_steps = self.enc.read()
        return self.left.update(l_steps, dt), self.right.update(r_steps, dt)

    def reset(self):
        """Stop both wheels and clear controller history."""
        l_steps, r_steps = self.enc.read()
        self.left.reset(l_steps)
        self.right.reset(r_steps)

    def close(self):
        """Stop the motors first, then put the driver to sleep."""
        self.left.motor.stop()
        self.right.motor.stop()
        self.slp.off()
        self.enc.close()