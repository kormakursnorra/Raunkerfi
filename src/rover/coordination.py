import board
import adafruit_icm20x
import adafruit_tcs34725
from gpiozero import MCP3008

STILL, FORWARD, LEFT, RIGHT, BACKWARD = "s", "f", "l", "r", "b"

INTEGRATION_TIME_MS = 24
COLOR_GAIN          = 4

WHITE_REFS = (1536, 1298, 712, 3562)
WHITE_MIN_BRIGHTNESS = 0.73   # darkest white seen: 0.81
WHITE_MAX_IMBALANCE  = 0.05   # most unbalanced white seen: 0.012
SATURATION_LIMIT = 7680

MAX_TURN_ANGLE   = 45.0       # degrees at full deflection
FULL_ERROR       = 0.40       # green/red share difference that gives MAX_TURN_ANGLE
FORWARD_DEADBAND = 5.0        # degrees; smaller turns are reported as FORWARD

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
    """Coordination controller for compass, distance- and color sensor."""

    def __init__(self):
        i2c = board.I2C() # Initialize I2C bus

        # Initialize all hardware components
        self.color_sensor = adafruit_tcs34725.TCS34725( i2c )
        self.imu_compass  = adafruit_icm20x.ICM20948( i2c, address=0x69 )
        self.infra_sensor = MCP3008( channel=0 )  # CE0 by default

        # Color sensor configurations
        self.color_sensor.integration_time = INTEGRATION_TIME_MS
        self.color_sensor.gain = COLOR_GAIN
        self.saturation = saturation_limit(self.color_sensor.integration_time)
        self.white_refs = WHITE_REFS  # override at runtime with calibrate_white()

        self.distance   = 0.0
        self.turn_angle = 0.0  # + for right turns, - for left turns. 0.0 for neutral
        self.direction  = ( STILL, self.turn_angle )
          
        self.last_raw      = None   # (r, g, b, clear) of the latest reading
        self.last_features = None   # color_features() of it, or None


    def reset(self):
        """Reset all units and forget their history."""
        self.imu_compass.reset()
        self.turn_angle = 0.0
        self.direction = (STILL, 0.0)

    def _calibrate_white(self, samples=10):
        """Optional: replace WHITE_REF with a fresh reading. Sensor must be over clean white."""
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
        """Measures and returns the distance from the average voltage of N sample."""
        avg_volts = sum(self.infra_sensor.value for _ in range(samples)) / samples * 3.3
        self.distance = 67.7 / (avg_volts + 0.144) - 5.6
        return avg_volts

    def read_color_features(self):
        """Return (brightness, imbalance, green_share, blue_share, red_share) or None.
         
        Channels are white-balanced against white_ref, so clean white gives
        ~1.0 per channel and equal shares of ~0.33. None means the reading is
        unreliable (dark/blocked or saturated).
        """
        r, g, b, clear = self.color_sensor.color_raw
        if clear <= 0 or clear >= self.saturation:
            return None
    
        wr, wg, wb, wc = self.white_refs
        rn, gn, bn = r / wr, g / wg, b / wb
        total = rn + gn + bn
        if total <= 0:
            return None
    
        brightness = clear / wc
        peak = max(rn, gn, bn)
        imbalance = (peak - min(rn, gn, bn)) / peak

        self.last_raw      = r, g, b, clear
        self.last_features = brightness, imbalance, gn / total, bn / total, rn / total
        return self.last_features

    def next_direction(self):
        """Takes a reading of the current track layout and finds the next direction.
 
        Line stripes from left to right: green | blue | red.
          over blue  -> centred, go forward
          over green -> robot is left of the line, turn right (+)
          over red   -> robot is right of the line, turn left (-)
          over white -> lost, keep turning the way we last turned
        """
        features = self.read_color_features()
        if features is None:
            return self.direction  # bad reading: keep the last command
 
        brightness, imbalance, green_share, _blue_share, red_share = features
 
        # Case 1: on white, the line is lost. Search toward where it was last seen.
        if brightness >= WHITE_MIN_BRIGHTNESS and imbalance <= WHITE_MAX_IMBALANCE:
            if self.turn_angle > 0:
                self.direction = (RIGHT, MAX_TURN_ANGLE)
            elif self.turn_angle < 0:
                self.direction = (LEFT, -MAX_TURN_ANGLE)
            else:
                self.direction = (STILL, 0.0)  # no idea which side it went; let caller decide
            return self.direction
 
        # Case 2: on the line. Steer proportionally to green vs red.
        error = green_share - red_share            # + = too far left, - = too far right
        scaled = max(-1.0, min(1.0, error / FULL_ERROR))
        self.turn_angle = scaled * MAX_TURN_ANGLE
 
        if abs(self.turn_angle) < FORWARD_DEADBAND:
            self.direction = (FORWARD, 0.0)
        elif self.turn_angle > 0:
            self.direction = (RIGHT, self.turn_angle)
        else:
            self.direction = (LEFT, self.turn_angle)
        return self.direction


    def shutdown(self):
        """Close/shut down all units."""
        self.color_sensor.active = False
        self.infra_sensor.close()
