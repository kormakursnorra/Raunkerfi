"""
closed-loop speed control, one PID per wheel.
 
Control law per wheel:
    duty = deadband + kf * target_speed  +  PID(target - measured)

The feedforward part does most of the work; the PID only corrects
the leftover error (load, battery sag, motor differences).
 
Speeds are in encoder steps per second, same units as calibrate.py.
"""

import csv
import math
from time import sleep, monotonic
 
from simple_pid import PID
from gpiozero import DigitalOutputDevice, Motor

from encoder_client import EncoderClient


# The daemon updates its counts in batches every 1/RATE s (25 ms by default).
# A loop faster than that sees 0 new steps in some ticks and a double batch in
# others, so the speed estimate jitters. Keep LOOP_DT >= 1/RATE, or raise RATE
# in encoderd.cpp (e.g. RATE = 200) if you want a faster loop.
LOOP_DT = 0.05   # 20 Hz control loop

# Same signs as in speed_calibration.py: positive = forward.
LEFT_SIGN = -1
RIGHT_SIGN = 1

class WheelController:
    """Feedforward + PID speed controller for a single wheel."""
    def __init__(self, wheel_args: dict, k_values: dict, start_steps: int,
                 filter_alpha=0.5):
        self.motor: Motor            = wheel_args["motor"]
        self.sign: int               = wheel_args["sign"]
        self.kf: float               = wheel_args["kf"]
        self.deadband: float         = wheel_args["deadband"]
        
        self.alpha = filter_alpha # 1.0 = no filtering

        # set sample_time=None to overwrite PIDs default timing so that we have control
        # over when calculations are done.
        #
        # set the output_limits=(-0.5, 0.5) caps PIDs contribution
        self.pid   = PID(k_values["kp"], k_values["ki"], k_values["kd"],
                        setpoint=0.0, sample_time=None, output_limits=(-0.1, 0.1)))

        self.target     = 0.0
        self.speed      = 0.0
        self.last_steps = self.sign * start_steps

    def set_target(self, steps_per_s):
        """Sets the target speed."""
        self.target       = steps_per_s
        self.pid.setpoint = steps_per_s

    def update(self, raw_steps, dt):
        """Updates and returns the duty cycle value.

        raw_steps is this wheel's count from the daemon. Both wheels are read
        in ONE request per loop so their counts come from the same instant.
        """
        steps           = self.sign * raw_steps
        raw_speed       = (steps - self.last_steps) / dt
        self.last_steps = steps

        self.speed = self.alpha * raw_speed + (1 - self.alpha) * self.speed

        if self.target == 0:
            self.pid.reset()
            self.motor.stop()
            return 0.0

        feedforward = self.kf * self.target + math.copysign(self.deadband, self.target)
        correction  = self.pid(self.speed, dt=dt)
        duty        = max(-1.0, min(1.0, feedforward + correction))
        
        self.motor.value = duty
        return duty

slp = DigitalOutputDevice('GPIO17')

left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)

# Encoders come from the encoderd daemon. No RotaryEncoder objects here.
enc = EncoderClient()

L_KF, L_DEADBAND = 0.000073, 0.022
R_KF, R_DEADBAND = 0.000065, 0.042  

KP = 0.00003 # the main tunable that relates the error between the current value to the desired value.
KI = 0.0001  # determines how past errors should influence the control effort.
KD = 0.0 # used to influence the effort by the rate of change of the error.

k_values = { "kp": KP, "ki": KI, "kd": KD }

l_args = {
    "motor": left,
    "sign": LEFT_SIGN,
    "kf": L_KF,
    "deadband": L_DEADBAND
}

r_args = {
    "motor": right,
    "sign": RIGHT_SIGN,
    "kf": R_KF,
    "deadband": R_DEADBAND
}

l0, r0 = enc.read()
left_ctrl  = WheelController(l_args, k_values, l0)
right_ctrl = WheelController(r_args, k_values, r0)

# Test profile: (time_s, target steps/s). Step changes reveal the response
PROFILE = [(0.0, 600), (3.0, 1200), (6.0, 800), (9.0, 0), (10.0, None)]

def target_at(t):
    current = 0
    for start, value in PROFILE:
        if t >= start:
            current = value
    return current

try:
    slp.on()        # wake the driver only once everything is set up
    sleep(0.01)
    with open("pid_log.csv", "w", newline="", encoding="utf-8") as f:
        log = csv.writer(f)
        log.writerow(["time", "target",
                      "left_speed", "left_duty", 
                      "right_speed", "right_duty"
                    ])

        t_start   = monotonic()
        t_last    = t_start
        next_tick = t_start

        while True:
            next_tick += LOOP_DT
            
            # Wait first, so dt is never zero on the first pass
            sleep(max(0.0, next_tick - monotonic()))
            
            t_now  = monotonic()
            dt     = t_now - t_last
            t_last = t_now
            time   = t_now - t_start

            target = target_at( time )
            if target is None:
                break

            left_ctrl.set_target(target)
            right_ctrl.set_target(target)

            l_steps, r_steps = enc.read()
            ld = left_ctrl.update(l_steps, dt)
            rd = right_ctrl.update(r_steps, dt)

            log.writerow([f"{time:.3f}", target, f"{left_ctrl.speed:.1f}",
                          f"{ld:.3f}", f"{right_ctrl.speed:.1f}", f"{rd:.3f}"])


finally:
    left.stop()     # stop the motors first, then put the driver to sleep
    right.stop()
    slp.off()
    enc.close()
