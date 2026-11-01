import csv
from datetime import datetime

FIELDS = [
    "t", "dt", "volts",
    "r", "g", "b", "c",                                  # raw colour reading
    "bright", "whiteness", "lineness",                   # features (blank = bad reading)
    "error", "direction", "angle",                       # steering decision
    "l_target_cmd", "r_target_cmd",                      # what we asked for
    "l_target", "r_target",                              # after the accel ramp
    "l_speed", "r_speed", "l_duty", "r_duty",            # what the wheels did
]


def default_log_path(prefix="marvin"):
    """A timestamped file name, e.g. marvin-20261031-152900.csv."""
    return datetime.now().strftime(f"{prefix}-%Y%m%d-%H%M%S.csv")


class RunLogger:
    """Writes one CSV row per tick. With path=None every call does nothing."""

    def __init__(self, path=None):
        self.path = path
        self._file = None
        self._writer = None
        if path:
            self._file = open(path, "w", newline="")
            self._writer = csv.writer(self._file)
            self._writer.writerow(FIELDS)

    @property
    def enabled(self):
        return self._writer is not None

    def tick(self, t, dt, volts, coordination, driver, direction, angle, cmd, duties):
        """Log one tick.

        coordination: CoordinationController (reads last_raw / last_features)
        driver:       DriveController (reads ramped targets and filtered speeds)
        cmd:          (left, right) targets asked for this tick
        duties:       (left, right) duties returned by driver.update()
        """
        if not self._writer:
            return

        raw = coordination.last_raw or ("",) * 4
        feats = coordination.last_features
        if feats is None:
            feats_cols, error = [""] * 3, ""
        else:
            bright, whiteness, lineness, err = feats
            feats_cols = [f"{bright:.3f}", f"{whiteness:.3f}", f"{lineness:.3f}"]
            error = f"{err:+.3f}"

        lw, rw = driver.left, driver.right
        self._writer.writerow([
            f"{t:.3f}", f"{dt:.4f}", f"{volts:.3f}",
            *raw, *feats_cols,
            error, direction, f"{angle:+.1f}",
            f"{cmd[0]:.0f}", f"{cmd[1]:.0f}",
            f"{lw.target:.0f}", f"{rw.target:.0f}",
            f"{lw.speed:.0f}", f"{rw.speed:.0f}",
            f"{duties[0]:.3f}", f"{duties[1]:.3f}",
        ])

    def close(self):
        if self._file:
            self._file.close()
            self._file = None
            self._writer = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()