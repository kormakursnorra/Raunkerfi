import board
import adafruit_tcs34725

from time import sleep

INTEGRATION_TIME_MS = 24
COLOR_GAIN          = 4

color_sensor = adafruit_tcs34725.TCS34725( board.I2C() )
color_sensor.integration_time = INTEGRATION_TIME_MS
color_sensor.gain = COLOR_GAIN

saturation_vals = []

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

def calibrate_white(samples=10):
    """Record the clean-white reference. Call with the sensor over clean white track."""
    totals = [0, 0, 0, 0]
    for _ in range(samples):
        for i, value in enumerate(color_sensor.color_raw):
            totals[i] += value
    ref = tuple(t / samples for t in totals)

    saturation = saturation_limit(color_sensor.integration_time)

    if ref[3] >= saturation:
        raise RuntimeError( "Lower gain or integration time." )
    if min(ref) <= 0:
        raise RuntimeError("A channel reads 0 on white. Check the LED and wiring.")

    saturation_vals.append(saturation)
    return ref


num_trials = 5
white_references = []

for i in range(num_trials):
    print(f"Starting scan {i} ")
    white_references.append(calibrate_white())
    print(f"Scan {i} done, move to another spot")
    sleep(5)

avg_white_ref = sum(w for w in white_references) / num_trials
avg_saturation = sum(s for s in saturation_vals) / num_trials

print(f"Average white refrence: {avg_white_ref}")
print(f"Average saturation value: {avg_saturation}")