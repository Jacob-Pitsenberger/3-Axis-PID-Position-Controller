"""
Lightweight logging utilities for recording control loop data.
Designed for robotics applications where high-frequency logging
must be efficient, structured, and easy to analyze later.
"""

import csv
import time
from pathlib import Path


class DataLogger:
    """
    CSV-based logger for recording control loop data.
    Each call to log() writes one row of structured data.
    """

    def __init__(self, filename="flight_log.csv", directory="logs"):
        # Create logs directory if needed
        self.log_dir = Path(directory)
        self.log_dir.mkdir(exist_ok=True)

        # Timestamped filename for uniqueness
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.filepath = self.log_dir / f"{timestamp}_{filename}"

        # File handle and CSV writer
        self.file = open(self.filepath, "w", newline="")
        self.writer = None
        self.header_written = False

    def log(self, data: dict):
        """
        Log a dictionary of values.
        Keys become column headers on first write.
        """
        if not self.header_written:
            self.writer = csv.DictWriter(self.file, fieldnames=list(data.keys()))
            self.writer.writeheader()
            self.header_written = True

        self.writer.writerow(data)

    def close(self):
        """Close the log file."""
        if not self.file.closed:
            self.file.close()