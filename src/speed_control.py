from signal import pause
from simple_pid import PID
from gpiozero import DigitalInputDevice, Motor, RotaryEncoder


candidates = [4, 5, 6, 16, 17, 20, 21, 22, 23, 24, 25, 26, 27]
pins = [DigitalInputDevice(p, pull_up=True) for p in candidates]
for p in pins:
    p.when_activated = p.when_deactivated = lambda d: print("GPIO", d.pin.number, d.value)
pause()

# FREQ = 1000

# K_P = 0.0
# K_I = 0.0
# K_D = 0.0


# pid = PID(Kp=K_P, Ki=K_I, Kd=K_D, setpoint=1.0)

# left = Motor('GPIO12', 'GPIO18', pwm=True)

# right = Motor('GPIO19', 'GPIO13', pwm=True)
