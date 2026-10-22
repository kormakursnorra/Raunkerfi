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
from gpiozero import DigitalOutputDevice, Motor, RotaryEncoder


LOOP_DT = 0.02   # 50 Hz control loop

class WheelController:
    """This class sets up a single wheel for """
    def __init__(self, wheel_args: dict, k_values: dict, filter_alpha=0.5):
        self.motor: Motor            = wheel_args["motor"]
        self.encoder: RotaryEncoder  = wheel_args["encoder"]
        self.kf: float               = wheel_args["kf"]
        self.deadband: float         = wheel_args["deadband"]
        
        self.alpha = filter_alpha # 1.0 = no filtering

        # set sample_time=None to overwrite PIDs default timing so that we have control
        # over when calculations are done.
        #
        # set the output_limits=(-0.5, 0.5) caps PIDs contribution
        self.pid   = PID(k_values["kp"], k_values["ki"], k_values["kd"],
                        setpoint=0.0, sample_time=None, output_limits=(-0.5, 0.5))

        self.target     = 0.0
        self.speed      = 0.0
        self.last_steps = wheel_args["encoder"].steps

    def set_target(self, steps_per_s):
        """Sets the target speed."""
        self.target       = steps_per_s
        self.pid.setpoint = steps_per_s

    def update(self, dt):
        """Updates and returns the duty cycle value."""
        steps           = self.encoder.steps
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
slp.on()

left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)

left_enc = RotaryEncoder('GPIO25', 'GPIO20', max_steps=0)
right_enc = RotaryEncoder('GPIO26', 'GPIO5', max_steps=0)


L_KF, L_DEADBAND = 0.0010, 0.08
R_KF, R_DEADBAND = 0.0010, 0.08

KP = 0.0 # the main tunable that relates the error between the current value to the desired value.
KI = 0.0 # determines how past errors should influence the control effort.
KD = 0.0 # used to influence the effort by the rate of change of the error.

k_values = { "kp": KP, "ki": KI, "kd": KD }

l_args = {
    "motor": left,
    "encoder": left_enc, 
    "kf": L_KF,
    "deadband": L_DEADBAND
}

r_args = {
    "motor": right,
    "encoder": right_enc, 
    "kf": R_KF,
    "deadband": R_DEADBAND
}

left_ctrl  = WheelController(l_args, k_values)
right_ctrl = WheelController(r_args, k_values)

# Test profile: (time_s, target steps/s). Step changes reveal the response
PROFILE = [(0.0, 300), (3.0, 600), (6.0, 200), (9.0, 0), (10.0, None)]

def target_at(t):
    current = 0
    for start, value in PROFILE:
        if t >= start:
            current = value
    return current

try:
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

            ld = left_ctrl.update(dt)
            rd = right_ctrl.update(dt)

            log.writerow([f"{time:.3f}", target, f"{left_ctrl.speed:.1f}",
                          f"{ld:.3f}", f"{right_ctrl.speed:.1f}", f"{rd:.3f}"])

            sleep(max(0.0, next_tick - monotonic()))


finally:
    slp.off()
    left.stop()
    right.stop()
