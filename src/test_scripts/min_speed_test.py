"""
Finds the lowest wheel speed the controller can hold reliably.

Run it with the robot ON THE FLOOR, not with the wheels in the air: load
changes stiction a lot. Trials alternate forward/backward so the robot
shuffles back and forth instead of driving away. Give it about 1 m of room.

Two kinds of trial per target speed:
  rest   start from standstill            (wheel waking up after a stop)
  decel  run at DECEL_FROM, then drop     (inside wheel slowing in a turn)

A wheel passes a trial when, during the measurement window:
  - no tick had (almost) zero raw speed  -> it never stalled
  - the mean speed is within MAX_ERR of the target
  - for 'rest', it started moving within MAX_START_S

Results: a table on screen, every tick in min_speed_log.csv.
"""

import csv
import statistics as st
from time import sleep, monotonic

from marvin.wheel_control import Drive, LOOP_DT

TARGETS = [100, 150, 200, 250, 300, 400, 500]   # steps/s
DECEL_FROM = 800       # steps/s, cruise speed before a 'decel' trial
CRUISE_S = 1.0         # time at DECEL_FROM before dropping
SETTLE_S = 1.0         # ignored part after reaching the target
MEASURE_S = 1.5        # part that is judged
PAUSE_S = 0.7          # standstill between trials

MAX_ERR = 0.20         # allowed |mean - target| / target
MAX_START_S = 0.5      # 'rest' trials must start moving this fast
STALL_FRAC = 0.2       # raw speed below this fraction of target = stall tick


def run_trial(drive, log, trial, mode, target):
    """Runs one trial and returns a result dict per wheel."""
    drive.reset()
    sleep(PAUSE_S)
    drive.reset()           # re-sync counts after the wheels came to rest

    pre = CRUISE_S if mode == "decel" else 0.0
    t_end = pre + SETTLE_S + MEASURE_S
    sign = 1 if target > 0 else -1

    samples = {"left": [], "right": []}
    start_time = {"left": None, "right": None}

    t_start = monotonic()
    t_last = t_start
    next_tick = t_start
    while True:
        next_tick += LOOP_DT
        sleep(max(0.0, next_tick - monotonic()))
        t_now = monotonic()
        dt = t_now - t_last
        t_last = t_now
        t = t_now - t_start
        if t >= t_end:
            break

        cmd = sign * DECEL_FROM if t < pre else target
        drive.set_targets(cmd, cmd, dt)
        ld, rd = drive.update(dt)

        for name, ctrl in (("left", drive.left), ("right", drive.right)):
            moving = abs(ctrl.raw_speed) > STALL_FRAC * abs(target)
            if moving and start_time[name] is None:
                start_time[name] = t
            if t >= pre + SETTLE_S:
                samples[name].append(ctrl.raw_speed * sign)

        log.writerow([trial, mode, target, f"{t:.3f}", f"{cmd}",
                      f"{drive.left.target:.1f}",
                      f"{drive.left.raw_speed:.1f}", f"{drive.left.speed:.1f}", f"{ld:.3f}",
                      f"{drive.right.raw_speed:.1f}", f"{drive.right.speed:.1f}", f"{rd:.3f}"])

    drive.reset()

    results = {}
    goal = abs(target)
    for name, sp in samples.items():
        mean = st.mean(sp)
        stalls = sum(1 for s in sp if s < STALL_FRAC * goal)
        err = (mean - goal) / goal
        cv = st.pstdev(sp) / mean if mean > 0 else float("inf")
        start = start_time[name]
        ok = stalls == 0 and abs(err) <= MAX_ERR
        if mode == "rest":
            ok = ok and start is not None and start <= MAX_START_S
        results[name] = dict(mean=mean, err=err, cv=cv, stalls=stalls,
                             n=len(sp), start=start, ok=ok)
    return results


def lowest_reliable(rows, mode):
    """Lowest target such that it and every higher target passed all trials
    of this mode (both wheels, both directions)."""
    best = None
    for target in sorted(TARGETS, reverse=True):
        if all(r["ok"] for (tg, m, _d, _w), r in rows.items()
               if tg == target and m == mode):
            best = target
        else:
            break
    return best


def main():
    drive = Drive()
    rows = {}
    try:
        drive.start()
        sleep(0.01)
        with open("min_speed_log.csv", "w", newline="", encoding="utf-8") as f:
            log = csv.writer(f)
            log.writerow(["trial", "mode", "target", "time", "command",
                          "ramped_target",
                          "left_raw", "left_speed", "left_duty",
                          "right_raw", "right_speed", "right_duty"])
            trial = 0
            for target in TARGETS:
                for mode in ("rest", "decel"):
                    for direction in (1, -1):
                        trial += 1
                        res = run_trial(drive, log, trial, mode, direction * target)
                        dname = "fwd" if direction > 0 else "back"
                        for wheel, r in res.items():
                            rows[(target, mode, dname, wheel)] = r
                        print(f"trial {trial:2d}: {target:4d} {mode:5s} {dname:4s}  "
                              + "  ".join(f"{w}={'ok' if r['ok'] else 'FAIL'}"
                                          for w, r in res.items()))
    finally:
        drive.close()

    print()
    print(f"{'target':>6} {'mode':5} {'dir':4} {'wheel':5} {'mean':>7} {'err%':>6} "
          f"{'cv%':>5} {'stall':>5} {'start_s':>7}  result")
    for (target, mode, dname, wheel), r in rows.items():
        start = f"{r['start']:.2f}" if r["start"] is not None else "never"
        if mode == "decel":
            start = "-"
        print(f"{target:6d} {mode:5} {dname:4} {wheel:5} {r['mean']:7.1f} "
              f"{100 * r['err']:6.1f} {100 * r['cv']:5.1f} "
              f"{r['stalls']:2d}/{r['n']:<2d} {start:>7}  "
              f"{'ok' if r['ok'] else 'FAIL'}")

    print()
    for mode, meaning, use in (
            ("decel", "slowing down while moving", "MIN_WHEEL_SPEED (steering clamp)"),
            ("rest", "starting from standstill", "lowest safe start speed")):
        best = lowest_reliable(rows, mode)
        if best is None:
            print(f"{mode}: nothing passed ({meaning}), even {max(TARGETS)} steps/s.")
        else:
            print(f"{mode}: lowest reliable {best} steps/s ({meaning}). "
                  f"Suggested {use}: {round(best * 1.25)} (25% margin)")
    print("Note: at low speed, one encoder step per tick is "
          f"{1 / LOOP_DT:.0f} steps/s, so part of the cv% is quantisation.")


if __name__ == "__main__":
    main()