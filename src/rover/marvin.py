import argparse
from time import sleep, monotonic

from .log_run import RunLogger, default_log_path
from .wheel_control import DriveController, LOOP_DT
from .coordination import CoordinationController, MAX_TURN_ANGLE, STILL

BASE_SPPED  = 375   # constant speed for following the line
STOP_VOLTS  = 2.52  # IR voltage at around 20 cm
STILL_TICKS = 10    # stop after this many consecutive STILL ticks  (0.5 s)

class Marvin:
    """Class representing the rover robot."""
    def __init__(self):
        # Initialize controller components
        self.driver = DriveController()
        try:
            self.coordination = CoordinationController()
        except Exception as e:
            print(f"ERROR occured when initializing CoordinationController: {e}")
            self.driver.shutdown()
            raise

        self.base_speed = BASE_SPPED

    def wheel_targets(self, turn_angle, max_turn=MAX_TURN_ANGLE):
        """Maps a turn angle (+ right, - left) to (left, right) wheel targets."""
        speed = max(-1.0, min(1.0, turn_angle / max_turn))  # -1 hard left .. +1 hard right
        return self.base_speed * (1 + speed), self.base_speed * (1 - speed)  # (left, right)

    def follow_line(self, log=None):
        """Main loop of the robot. """
        log = log or RunLogger(None)   # disabled logger: tick() does nothing
        try:
            self.driver.start()   # wake the driver only once everything is set up
            sleep(0.01)

            t_start  = monotonic()
            t_last   = t_start
            nxt_tick = t_start
            still_count = 0

            while True:
                # Fixed-rate tick. If we overran, re-anchor instead of
                # running several ticks back to back.
                nxt_tick += LOOP_DT
                now = monotonic()
                nxt_tick = max(nxt_tick, now)
                # Wait first, so dtime is never zero on the first pass
                sleep(max(0.0, nxt_tick - now))

                # for accuracy, distance is measured by voltage
                distance = self.coordination.measure_distance()
                if distance >= STOP_VOLTS:
                    return f"Object at about 20 cm  ({distance:.2f} V)"


                direction, turn_angle = self.coordination.next_direction()

                t_now  = monotonic()
                dtime  = t_now - t_last
                t_last = t_now

                left_target, right_target = 0.0, 0.0

                if direction == STILL:
                    still_count += 1
                    if still_count >= STILL_TICKS:
                        return "Line not found"
                else:
                    still_count = 0
                    left_target, right_target = self.wheel_targets( turn_angle )

                self.driver.set_targets(left_target, right_target, dtime)
                duties = self.driver.update(dtime)

                log.tick(t_now - t_start, dtime, distance, self.coordination,
                         self.driver, direction, turn_angle, (left_target, right_target), duties)



        except KeyboardInterrupt:
            return "Keybaord interruption"

        finally:
            self.driver.shutdown()
            self.coordination.shutdown()

def main():
    parser = argparse.ArgumentParser(description="Run Marvin's line follower.")
    parser.add_argument("--log", help="CSV log path (default: marvin-<time>.csv)")
    parser.add_argument("--no-log", action="store_true", help="disable logging")
    args = parser.parse_args()

    log_path = None if args.no_log else (args.log or default_log_path())

    marvin = Marvin()
    with RunLogger(log_path) as log:
        print("Stopped: ", marvin.follow_line(log))
    
    if log_path:
        print("Log written to", log_path)





if __name__ == "__main__":
    main()