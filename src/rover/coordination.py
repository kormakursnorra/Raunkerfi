import board
import adafruit_tcs34725
from gpiozero import MCP3008

STILL, FORWARD, LEFT, RIGHT, BACKWARD = "s", "f", "l", "r", "b"

INTEGRATION_TIME_MS = 24
COLOR_GAIN          = 4

WHITE_REFS = (1536, 1298, 712, 3562)   # (r, g, b, clear) over white canvas

# Whiteness = smallest white-balanced channel (min of r/wr, g/wg, b/wb).
# White canvas lifts ALL channels, so whiteness is high. Black tape and any
# coloured stripe (red, green, blue, ...) leave at least one channel low, so
# whiteness is low. Stripe colour and stripe order do not matter.
# Values from marvin-20261031-154850.csv:
#   coloured line: 0.08 .. 0.19     white canvas: 0.55 .. 1.06
LINE_WHITENESS  = 0.10        # typical reading fully on the line
WHITE_WHITENESS = 0.55        # dimmest white seen

# Edge following: the robot tracks ONE edge of the line, not its centre.
#   LEFT_EDGE:  white on the robot's left, line on its right
#   RIGHT_EDGE: line on the robot's left, white on its right
LEFT_EDGE, RIGHT_EDGE = +1, -1
EDGE = LEFT_EDGE

MAX_TURN_ANGLE   = 45.0       # degrees at full deflection (full pivot on one wheel)
STEER_GAIN       = 0.5        # fraction of MAX_TURN_ANGLE used at full error. 1.0 = old
                              # behaviour (pivot every time): wobbled with a 0.5 s period
                              # while the circle only needed ~8 deg on average.
FORWARD_DEADBAND = 5.0        # degrees; smaller turns are labelled FORWARD (angle is kept)
SEARCH_TICKS     = 14         # all-white this long (0.7 s): assume we crossed the line, switch edge
LOST_TICKS       = 40         # report STILL after this many all-white readings in a row

# Damping. The wheels answer a steering command ~3 ticks (150 ms) late
# (marvin-20261101-172530.csv), which turns a pure P controller into a
# 0.45 s limit cycle. The D term reacts to lineness *changing*, before the
# error itself gets big. Start here and tune: raise until the wobble shrinks,
# back off if it gets twitchy on sensor noise.
STEER_KD         = 0.6        # same units as the normalised error (error/0.5)

# Tight turns. In the log the sensor sat fully on the line for ~30 ticks while
# steering was capped at STEER_GAIN, then slid out the far side. While pinned,
# grow the gain toward a full pivot instead of staying capped.
PINNED_LINENESS  = 0.9        # "fully on the line"
BOOST_AFTER      = 4          # normal wobble pins for <= 4 ticks; boost only beyond that
BOOST_PER_TICK   = 0.08       # extra gain per tick past BOOST_AFTER (0.5 -> 1.0 in ~6 ticks)
MAX_GAIN         = 1.0        # 1.0 = full MAX_TURN_ANGLE = pivot on the inner wheel

# Crossing detection. Normal wobble pins the sensor for at most 4 ticks.
# A longer pin followed by white means we went across the whole line and are
# on its far side, where the current edge logic steers the wrong way.
CROSS_TICKS      = 6          # pinned at least this long ...
CROSS_RECENT     = 8          # ... and the pin ended at most this many ticks ago


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


class CoordinationController:
    """Coordination controller for the distance and colour sensors."""

    def __init__(self, edge=EDGE):
        i2c = board.I2C()

        self.color_sensor = adafruit_tcs34725.TCS34725(i2c)
        self.infra_sensor = MCP3008(channel=0)  # CE0 by default

        self.color_sensor.integration_time = INTEGRATION_TIME_MS
        self.color_sensor.gain = COLOR_GAIN
        self.saturation = saturation_limit(self.color_sensor.integration_time)
        self.white_refs = WHITE_REFS  # override at runtime with calibrate_white()

        self.edge = edge
        self.distance = 0.0
        self.turn_angle = 0.0  # + right, - left
        self.direction = (STILL, 0.0)
        self.white_count = 0
        self.prev_error = 0.0
        self.pin_streak = 0     # consecutive pinned ticks right now
        self.last_streak = 0    # length of the most recent finished pin
        self.since_pin = 99     # ticks since that pin ended

        self.last_raw      = None   # (r, g, b, clear) of the latest reading
        self.last_features = None   # (brightness, whiteness, lineness, error) or None

    def reset(self):
        """Forget steering history."""
        self.turn_angle = 0.0
        self.direction = (STILL, 0.0)
        self.white_count = 0
        self.prev_error = 0.0
        self.pin_streak = 0
        self.last_streak = 0
        self.since_pin = 99

    def calibrate_white(self, samples=10):
        """Replace WHITE_REFS with a fresh reading. Sensor must be over clean white."""
        totals = [0, 0, 0, 0]
        for _ in range(samples):
            for i, value in enumerate(self.color_sensor.color_raw):
                totals[i] += value
        ref = tuple(t / samples for t in totals)

        if ref[3] >= self.saturation:
            raise RuntimeError(
                f"White saturates the sensor (C={ref[3]:.0f}, limit={self.saturation}). "
                "Lower gain or integration time."
            )
        if min(ref) <= 0:
            raise RuntimeError("A channel reads 0 on white. Check the LED and wiring.")

        self.white_refs = ref
        return ref

    def measure_distance(self, samples=20):
        """Returns the average IR voltage of N samples (also stores a cm estimate)."""
        avg_volts = sum(self.infra_sensor.value for _ in range(samples)) / samples * 3.3
        self.distance = 67.7 / (avg_volts + 0.144) - 5.6
        return avg_volts

    def read_line(self):
        """Return lineness in [0, 1] (0 = white canvas, 1 = on the line), or None.

        None means the reading is unreliable (blocked or saturated).
        """
        r, g, b, clear = self.color_sensor.color_raw
        self.last_raw = r, g, b, clear
        if clear <= 0 or clear >= self.saturation:
            self.last_features = None
            return None

        wr, wg, wb, wc = self.white_refs
        whiteness = min(r / wr, g / wg, b / wb)
        lineness = (WHITE_WHITENESS - whiteness) / (WHITE_WHITENESS - LINE_WHITENESS)
        lineness = max(0.0, min(1.0, lineness))

        self.last_features = (clear / wc, whiteness, lineness, 0.5 - lineness)
        return lineness

    def next_direction(self):
        """Read the sensor and return (direction, turn_angle).

        Proportional edge following. The target is the edge (lineness 0.5):
          more white than target -> turn toward the line side
          more line than target  -> turn toward the white side
        Fully white for SEARCH_TICKS readings -> switch to the other edge.
        Fully white for LOST_TICKS readings in a row -> STILL (line lost).
        """
        lineness = self.read_line()
        if lineness is None:
            return self.direction  # bad reading: keep the last command

        # Track how long we have been fully on the line.
        if lineness >= PINNED_LINENESS:
            self.pin_streak += 1
            self.since_pin = 0
        else:
            if self.pin_streak:
                self.last_streak = self.pin_streak
            self.pin_streak = 0
            self.since_pin += 1

        if lineness <= 0.0:
            if (self.white_count == 0 and self.last_streak >= CROSS_TICKS
                    and self.since_pin <= CROSS_RECENT):
                # Long pin, then white: we slid across the line. Switch edge
                # now instead of waiting SEARCH_TICKS turning the wrong way.
                self.edge = -self.edge
                self.last_streak = 0
                self.prev_error = 0.0   # no D kick from the flip
            self.white_count += 1
            if self.white_count == SEARCH_TICKS:
                # Still white after turning toward the line for a while: we most
                # likely overshot across the line and are on its far side, where
                # turning "toward the line" points away from it. Follow the
                # other edge instead, which turns us back. Either edge works.
                self.edge = -self.edge
            if self.white_count >= LOST_TICKS:
                self.turn_angle = 0.0
                self.direction = (STILL, 0.0)
                return self.direction
        else:
            self.white_count = 0

        # error > 0: too far onto white. LEFT_EDGE has the line on the right,
        # so a positive error means turn right (+).
        error = (0.5 - lineness) / 0.5           # normalised to [-1, 1]
        d_error = error - self.prev_error
        self.prev_error = error

        # Stuck on either side (on the line, or on white) -> steer harder.
        stuck = max(self.pin_streak, self.white_count) - BOOST_AFTER
        gain = min(MAX_GAIN, STEER_GAIN + BOOST_PER_TICK * max(0, stuck))
        u = gain * error + STEER_KD * d_error
        u = max(-1.0, min(1.0, u))
        self.turn_angle = self.edge * u * MAX_TURN_ANGLE

        if abs(self.turn_angle) < FORWARD_DEADBAND:
            self.direction = (FORWARD, self.turn_angle)
        elif self.turn_angle > 0:
            self.direction = (RIGHT, self.turn_angle)
        else:
            self.direction = (LEFT, self.turn_angle)
        return self.direction

    def shutdown(self):
        """Close/shut down all units."""
        self.color_sensor.active = False
        self.infra_sensor.close()