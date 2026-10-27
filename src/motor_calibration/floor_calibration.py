"""
open-loop calibration on the floor.

Run it on the surface the line will be on, with a charged battery, and write
down the battery voltage. Steps alternate forward/backward so the robot
shuffles back and forth instead of driving away. Give it 1-2 m of room.

Part 1, break-away: from standstill, slowly raise the duty until each wheel
starts moving. This is the duty to use as KICK_DUTY in wheel_control.py.

Part 2, sweep: for each duty, kick the wheels free, hold the duty, measure the
running speed. Fits duty = deadband + kf * speed per wheel and per direction.

Output: lines to paste into wheel_control.py.
"""

from time import sleep, monotonic
from gpiozero import DigitalOutputDevice, Motor

from marvin.encoder_client import EncoderClient

DUTIES = [0.04, 0.06, 0.08, 0.10, 0.12, 0.15, 0.20]

KICK = 0.15          # duty used to break the wheels free before each step
KICK_S = 0.15        # seconds
SETTLE_S = 0.8       # let the speed settle at the step's duty
MEASURE_S = 1.0
PAUSE_S = 0.5        # standstill between steps

BREAK_STEP = 0.005   # break-away test: duty increment ...
BREAK_EVERY = 0.1    # ... every this many seconds
BREAK_MOVING = 50.0  # steps/s that counts as "moving"
BREAK_MAX = 0.25     # give up above this duty

# The daemon can only process STEPS * RATE / 4 = 2100 steps/s per wheel with
# its default settings. Above that its queue backs up and the counts lag.
DAEMON_LIMIT = 2100

LEFT_SIGN = -1
RIGHT_SIGN = 1

slp = DigitalOutputDevice('GPIO17')

left = Motor(forward='GPIO12', backward='GPIO18', pwm=True)
right = Motor(forward='GPIO19', backward='GPIO13', pwm=True)

# Encoders are read from the encoderd daemon, NOT gpiozero.
# Do not create RotaryEncoder objects on GPIO25/20/26/5: the daemon owns them.
enc = EncoderClient()


def read_signed():
    """Encoder counts with positive = forward for both wheels."""
    l, r = enc.read()
    return LEFT_SIGN * l, RIGHT_SIGN * r


def measure(seconds=1.0) -> tuple[float, float]:
    """Returns the measured speed of the left and right wheels in steps/s."""
    left0, right0 = read_signed()
    time0 = monotonic()
    sleep(seconds)
    left1, right1 = read_signed()
    dt = monotonic() - time0
    return (left1 - left0) / dt, (right1 - right0) / dt


def stop_both():
    left.stop()
    right.stop()


def breakaway(direction):
    """Raises the duty from 0 until each wheel moves. Returns (left, right)
    break-away duty, or None for a wheel that never moved."""
    found = {"left": None, "right": None}
    motors = {"left": left, "right": right}
    duty = 0.0
    l_prev, r_prev = read_signed()
    t_prev = monotonic()
    while duty < BREAK_MAX and None in found.values():
        duty += BREAK_STEP
        for name, m in motors.items():
            if found[name] is None:
                m.value = direction * duty
        sleep(BREAK_EVERY)
        l_now, r_now = read_signed()
        t_now = monotonic()
        dt = t_now - t_prev
        speeds = {"left": (l_now - l_prev) / dt, "right": (r_now - r_prev) / dt}
        l_prev, r_prev, t_prev = l_now, r_now, t_now
        for name, s in speeds.items():
            if found[name] is None and abs(s) > BREAK_MOVING:
                found[name] = duty
                motors[name].stop()   # this wheel is done
    stop_both()
    return found["left"], found["right"]


def fit_line(points, min_speed=20.0, max_speed=1800.0):
    """Least-squares fit of duty = deadband + kf * speed.

    points are (duty, speed) with both >= 0. Only uses points where the
    wheel moved (above min_speed) and the daemon was not saturated.
    """
    pts = [(s, d) for d, s in points if min_speed < s < max_speed]  # (speed, duty)

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
    deadband = mean_d - kf * mean_s     # intercept of the fitted line
    return kf, deadband, n


# data[wheel][direction] = list of (duty, speed), both as positive numbers
data = {w: {1: [], -1: []} for w in ("L", "R")}
breaks = {w: [] for w in ("L", "R")}

try:
    slp.on()        # wake the driver, otherwise the Motor objects do nothing
    sleep(0.01)

    print("Break-away test (duty at which each wheel starts moving)")
    for direction in (1, -1):
        sleep(PAUSE_S)
        bl, br = breakaway(direction)
        breaks["L"].append(bl)
        breaks["R"].append(br)
        dname = "fwd " if direction > 0 else "back"
        fmt = lambda b: f"{b:.3f}" if b is not None else "never"
        print(f"  {dname}: left {fmt(bl)}, right {fmt(br)}")

    print()
    print("duty, dir,  left_steps_per_s, right_steps_per_s")
    for duty in DUTIES:
        for direction in (1, -1):
            sleep(PAUSE_S)
            left.value = right.value = direction * KICK
            sleep(KICK_S)
            left.value = right.value = direction * duty
            sleep(SETTLE_S)
            l, r = measure(MEASURE_S)
            stop_both()
            data["L"][direction].append((duty, direction * l))
            data["R"][direction].append((duty, direction * r))
            dname = "fwd " if direction > 0 else "back"
            print(f"{duty:.2f}, {dname}, {l:9.1f}, {r:9.1f}")
finally:
    stop_both()     # stop the motors first, then put the driver to sleep
    slp.off()
    enc.close()

print()
for name in ("L", "R"):
    top = max(s for d in (1, -1) for _, s in data[name][d])
    if top > 0.9 * DAEMON_LIMIT:
        print(f"WARNING {name}: top speed {top:.0f} steps/s is near the daemon "
              f"limit of {DAEMON_LIMIT}. Raise STEPS in encoderd.cpp.")

print("Fits per direction (if fwd and back differ a lot, tell me):")
results = {}
for name in ("L", "R"):
    for direction, dname in ((1, "fwd "), (-1, "back")):
        try:
            kf, db, n = fit_line(data[name][direction])
            print(f"  {name} {dname}: kf {kf:.6f}, deadband {db:.3f}  ({n} points)")
        except ValueError as e:
            print(f"  {name} {dname}: {e}. Check wiring, slp, or {name}_SIGN.")
    try:
        results[name] = fit_line(data[name][1] + data[name][-1])
    except ValueError:
        results[name] = None

print()
print("Paste into wheel_control.py (both directions combined):")
for name in ("L", "R"):
    if results[name] is not None:
        kf, db, n = results[name]
        print(f"{name}_KF, {name}_DEADBAND = {kf:.6f}, {db:.3f}  # fit from {n} points")
found = [b for w in ("L", "R") for b in breaks[w] if b is not None]
if found:
    print(f"KICK_DUTY = {max(found) + 0.01:.3f}  # highest break-away + 0.01")
missing = [w for w in ("L", "R") if None in breaks[w]]
if missing:
    print(f"WARNING: {', '.join(missing)} never broke free below {BREAK_MAX} duty.")