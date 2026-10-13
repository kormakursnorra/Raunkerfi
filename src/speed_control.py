from simple_pid import PID
from gpiozero import DigitalInputDevice, Motor, RotaryEncoder

K_P = 0.0
K_I = 0.0
K_D = 0.0


pid = PID(Kp=K_P, Ki=K_I, Kd=K_D, setpoint=1.0)

left = Motor('GPIO12', 'GPIO18', pwm=True)
right = Motor('GPIO19', 'GPIO13', pwm=True)

left_enc = RotaryEncoder('GPIO25', 'GPIO20', max_steps=0)
right_enc = RotaryEncoder('GPIO26', 'GPIO5', max_steps=0)


