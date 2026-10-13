from simple_pid import PID
from gpiozero import Device, Motor, Robot, RotaryEncoder

# FREQ = 1000

# K_P = 0.0
# K_I = 0.0
# K_D = 0.0

# pid = PID(Kp=K_P, Ki=K_I, Kd=K_D, setpoint=1.0)

# left = Motor('GPIO12', 'GPIO18', pwm=True)

# right = Motor('GPIO19', 'GPIO13', pwm=True)


print(Device.pin_factory)

# wheels = Robot(
# )

# for m in (wheels.left, wheels.right):
#     m.forward_device.frequency = m.backward_device.frequency = FREQ

# print(left.forward_device.frequency)
