import time
from dataclasses import dataclass


@dataclass
class PIDConfig:
    """Configuration for a PID controller."""
    kp: float
    ki: float
    kd: float
    output_limits: tuple = (-100, 100)  # RC command range
    integral_limits: tuple = (-50, 50)  # Anti-windup clamp
    derivative_filter_alpha: float = 0.7  # Derivative smoothing factor


class PID:
    """
    Reusable PID controller with:
    - Anti-windup (integral clamping)
    - Derivative filtering (exponential smoothing)
    - Output clamping
    """

    def __init__(self, config: PIDConfig):
        self.config = config

        # Internal state
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_derivative = 0.0  # For filtered derivative

    def reset(self):
        """Reset internal PID state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.prev_derivative = 0.0

    def compute(self, setpoint, measurement):
        """
        Compute PID output.
        setpoint: desired value
        measurement: current value
        """
        now = time.time()

        # Compute dt
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time

        self.prev_time = now

        # Error term
        error = setpoint - measurement

        # Proportional
        p = self.config.kp * error

        # Integral with anti-windup
        if dt > 0:
            self.integral += error * dt
            self.integral = max(self.config.integral_limits[0],
                                min(self.integral, self.config.integral_limits[1]))
        i = self.config.ki * self.integral

        # Derivative with filtering
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

        # Save error for next iteration
        self.prev_error = error

        # PID output
        output = p + i + d

        # Clamp output to limits
        output = max(self.config.output_limits[0],
                     min(output, self.config.output_limits[1]))

        return output


# ---------------------------------------------------------
# Pre-configured PID controllers for X, Y, and Z axes
# (You will tune these values during testing)
# ---------------------------------------------------------

pid_x = PID(PIDConfig(
    kp=40.0,
    ki=0.0,
    kd=20.0,
    output_limits=(-100, 100),
    integral_limits=(-30, 30),
    derivative_filter_alpha=0.6
))

pid_y = PID(PIDConfig(
    kp=40.0,
    ki=0.0,
    kd=20.0,
    output_limits=(-100, 100),
    integral_limits=(-30, 30),
    derivative_filter_alpha=0.6
))

pid_z = PID(PIDConfig(
    kp=60.0,
    ki=0.0,
    kd=30.0,
    output_limits=(-100, 100),
    integral_limits=(-20, 20),
    derivative_filter_alpha=0.7
))