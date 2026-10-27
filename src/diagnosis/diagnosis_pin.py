"""
Raw pin test. Bypasses Motor and RotaryEncoder completely.
Part 1: watch the encoder pins directly while you turn the wheels by hand.
Part 2: hold each motor pin HIGH so you can measure it with a multimeter.
"""
from time import sleep
from gpiozero import DigitalInputDevice, DigitalOutputDevice

ENC_PINS = {'L_A': 'GPIO25', 'L_B': 'GPIO20', 'R_A': 'GPIO26', 'R_B': 'GPIO5'}
MOTOR_PINS = {'L_fwd': 'GPIO12', 'L_back': 'GPIO18',
              'R_fwd': 'GPIO19', 'R_back': 'GPIO13'}

# ---------- Part 1: encoder pins ----------
inputs = {n: DigitalInputDevice(p, pull_up=True) for n, p in ENC_PINS.items()}
edges = {n: 0 for n in inputs}


def make_counter(name):
    def count():
        edges[name] += 1
    return count


for name, dev in inputs.items():
    dev.when_activated = make_counter(name)
    dev.when_deactivated = make_counter(name)

print("Raw encoder pin levels at rest (1 = high, 0 = low):")
for name, dev in inputs.items():
    print(f"  {name} ({ENC_PINS[name]}): {dev.pin.state}")

input("\nPress Enter, then turn BOTH wheels by hand for 5 s.")
sleep(5)
print("Edges seen (0 means that pin never changed):")
for name, n in edges.items():
    print(f"  {name}: {n}")

for dev in inputs.values():
    dev.close()

# ---------- Part 2: motor pins ----------
print("\nMultimeter: black probe on a Pi GND pin, red probe on the pin named.")
for name, pin in MOTOR_PINS.items():
    out = DigitalOutputDevice(pin)
    input(f"\nPress Enter to set {name} ({pin}) HIGH for 8 s.")
    out.on()
    print(f"  {pin} is HIGH now. Expect about 3.3 V on it.")
    print("  Also check the matching driver OUTPUT terminal for motor voltage.")
    sleep(8)
    out.off()
    out.close()

print("\nDone.")