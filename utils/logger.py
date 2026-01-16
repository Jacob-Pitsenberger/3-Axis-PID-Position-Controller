import csv
import time
from pathlib import Path
from utils.log_profiles import FULL_PID_PROFILE, SUPERVISORY_PROFILE

class DataLogger:
    def __init__(self, filename="flight_log.csv", directory="logs", mode="full_pid"):
        self.log_dir = Path(directory)
        self.log_dir.mkdir(exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = self.log_dir / f"{timestamp}_{filename}"

        self.file = open(self.filepath, "w", newline="")

        # Select profile
        if mode == "full_pid":
            self.fields = FULL_PID_PROFILE
        elif mode == "supervisory":
            self.fields = SUPERVISORY_PROFILE
        else:
            raise ValueError(f"Unknown logging mode: {mode}")

        # Write header immediately
        self.writer = csv.DictWriter(self.file, fieldnames=list(self.fields.keys()))
        self.writer.writeheader()

    def log_frame(self, context):
        """Log a single control-loop frame using the selected profile."""
        try:
            row = {name: extractor(context) for name, extractor in self.fields.items()}
            self.writer.writerow(row)
        except Exception as e:
            print(f"[LOG ERROR] Failed to write row: {e}")

    def close(self):
        if not self.file.closed:
            self.file.close()