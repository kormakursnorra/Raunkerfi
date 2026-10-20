"""
Motor + encoder diagnostics. Wheels off the ground.
Each test asks you to watch/listen and press Enter to continue.
"""
from time import sleep
from gpiozero import Device, Motor, RotaryEncoder
 
left = Motor('GPIO12', 'GPIO18', pwm=True)
right = Motor('GPIO19', 'GPIO13', pwm=True)
left_enc = RotaryEncoder('GPIO25', 'GPIO20', max_steps=0)
right_enc = RotaryEncoder('GPIO26', 'GPIO5', max_steps=0)
 
print("Pin factory in use:", type(Device.pin_factory).__name__)
 
try:
    # Test 1: encoders alone, no motor power involved
    input("\nTEST 1: Spin each wheel BY HAND for ~5 s after pressing Enter.")
    l0, r0 = left_enc.steps, right_enc.steps
    sleep(5)
    print(f"  left moved {left_enc.steps - l0} steps, right moved {right_enc.steps - r0} steps")
 
    # Test 2: full power, no PWM involved (duty = 1 means pin held fully on)
    for name, m in (("LEFT", left), ("RIGHT", right)):
        input(f"\nTEST 2: {name} motor FULL forward for 2 s. Press Enter.")
        m.forward(1.0)
        sleep(2)
        m.stop()
        input(f"  Now {name} motor FULL backward for 2 s. Press Enter.")
        m.backward(1.0)
        sleep(2)
        m.stop()
finally:
    left.stop()
    right.stop()
