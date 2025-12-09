"""
Base PID controller implementation with:
- Anti-windup (integral clamping)
- Derivative filtering (exponential smoothing)
- Output clamping
"""

import time
from dataclasses import dataclass


@dataclass
class PIDConfig:
    """Configuration for a PID controller."""
    kp: float
    ki: float
    kd: float
    output_limits: tuple = (-100, 100)       # RC command range
    integral_limits: tuple = (-50, 50)       # Anti-windup clamp
    derivative_filter_alpha: float = 0.7     # Derivative smoothing factor


class PID:
    def __init__(self, config: PIDConfig):
        self.config = config
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_derivative = 0.0

    def reset(self):
        """Reset internal PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_derivative = 0.0

    def compute(self, setpoint, measurement):
        """Compute PID output."""
        now = time.time()
        dt = 0.0 if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        error = setpoint - measurement
        p = self.config.kp * error

        if dt > 0:
            self.integral += error * dt
            self.integral = max(self.config.integral_limits[0],
                                min(self.integral, self.config.integral_limits[1]))
        i = self.config.ki * self.integral

        if dt > 0:
            raw_derivative = (error - self.prev_error) / dt
            alpha = self.config.derivative_filter_alpha
            self.prev_derivative = (
                alpha * raw_derivative +
                (1 - alpha) * self.prev_derivative
            )
        else:
            self.prev_derivative = 0.0

        d = self.config.kd * self.prev_derivative
        self.prev_error = error

        output = p + i + d
        output = max(self.config.output_limits[0],
                     min(output, self.config.output_limits[1]))

        return output