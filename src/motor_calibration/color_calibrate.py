"""Measure white references for coordination.py.

Run with the sensor mounted on the rover, at its real height, LED on.
Scans clean white spots first, then dirty white spots, and prints values
to hardcode: WHITE_REF, plus suggested WHITE_MIN_BRIGHTNESS and
WHITE_MAX_IMBALANCE.
"""
import time

import board
import adafruit_tcs34725

INTEGRATION_TIME_MS = 24
COLOR_GAIN          = 4
SAMPLES_PER_SPOT    = 10
CLEAN_SPOTS         = 5
DIRTY_SPOTS         = 5
MARGIN              = 0.9   # safety margin applied to the suggested thresholds


def saturation_limit(integration_time_ms):
    """Usable clear-channel ceiling per the ams DN40 app note.

    1024 counts per 2.4 ms step, capped at 65535, minus 25% ripple margin
    below 150 ms (same rule the Adafruit library uses).
    """
    steps = max(1, round(integration_time_ms / 2.4))
    limit = min(65535, 1024 * steps)
    if integration_time_ms < 150:
        limit -= limit // 4
    return limit


def read_average(sensor, samples):
    """Element-wise average of `samples` raw RGBC readings."""
    totals = [0, 0, 0, 0]
    for _ in range(samples):
        for i, value in enumerate(sensor.color_raw):
            totals[i] += value
    return tuple(t / samples for t in totals)


def average_tuples(tuples):
    n = len(tuples)
    return tuple(sum(t[i] for t in tuples) / n for i in range(4))


def scan_spots(sensor, label, count, saturation):
    """Scan `count` spots, retrying any that saturate or read zero."""
    refs = []
    while len(refs) < count:
        input(f"[{label} {len(refs) + 1}/{count}] Place the rover on a {label} spot, then press Enter...")
        ref = read_average(sensor, SAMPLES_PER_SPOT)
        r, g, b, c = ref
        if c >= saturation:
            print(f"  Saturated (C={c:.0f} >= {saturation}). Lower gain or integration time. Retry.")
            continue
        if min(ref) <= 0:
            print("  A channel read 0. Check the LED and wiring. Retry.")
            continue
        print(f"  R={r:.0f} G={g:.0f} B={b:.0f} C={c:.0f}")
        refs.append(ref)
    return refs


def features(ref, white):
    """Brightness and imbalance of `ref` relative to the white reference."""
    r, g, b, c = ref
    wr, wg, wb, wc = white
    rn, gn, bn = r / wr, g / wg, b / wb
    peak = max(rn, gn, bn)
    return c / wc, (peak - min(rn, gn, bn)) / peak


def main():
    sensor = adafruit_tcs34725.TCS34725(board.I2C())
    sensor.integration_time = INTEGRATION_TIME_MS
    sensor.gain = COLOR_GAIN

    print(f"Integration time actually set: {sensor.integration_time} ms, gain: {sensor.gain}")
    saturation = saturation_limit(sensor.integration_time)
    print(f"Saturation limit (computed, not measured): {saturation}")

    # Discard a reading that may have been integrated with the old settings.
    time.sleep(2 * sensor.integration_time / 1000)
    _ = sensor.color_raw

    clean = scan_spots(sensor, "clean white", CLEAN_SPOTS, saturation)
    white_ref = average_tuples(clean)

    dirty = scan_spots(sensor, "dirty white", DIRTY_SPOTS, saturation)

    print("\nPer-spot features relative to clean white:")
    all_features = []
    for label, refs in (("clean", clean), ("dirty", dirty)):
        for i, ref in enumerate(refs, 1):
            brightness, imbalance = features(ref, white_ref)
            all_features.append((brightness, imbalance))
            print(f"  {label} {i}: brightness={brightness:.2f} imbalance={imbalance:.3f}")

    min_brightness = min(f[0] for f in all_features)
    max_imbalance = max(f[1] for f in all_features)

    print("\nValues for coordination.py:")
    print(f"WHITE_REF = ({white_ref[0]:.0f}, {white_ref[1]:.0f}, {white_ref[2]:.0f}, {white_ref[3]:.0f})")
    print(f"WHITE_MIN_BRIGHTNESS = {min_brightness * MARGIN:.2f}   # darkest white seen: {min_brightness:.2f}")
    print(f"WHITE_MAX_IMBALANCE  = {max_imbalance / MARGIN:.3f}  # most unbalanced white seen: {max_imbalance:.3f}")
    print(f"SATURATION_LIMIT = {saturation}")
    print("\nCheck these against the green, blue and red stripes: none of them should")
    print("pass BOTH white thresholds, or the rover will think it lost the line.")


if __name__ == "__main__":
    main()