"""
Reusable filtering utilities for smoothing noisy sensor data.
These functions are drone-agnostic and can be used across
state estimation, PID control, and data logging.
"""

def low_pass_filter(previous_value, new_value, alpha):
    """
    Basic low-pass filter.
    alpha: smoothing factor in [0, 1]
           - closer to 1 = more responsive
           - closer to 0 = more smoothing
    """
    return alpha * new_value + (1 - alpha) * previous_value


def exponential_moving_average(previous_value, new_value, alpha):
    """
    Exponential Moving Average (EMA).
    Identical to a low-pass filter but semantically clearer
    when used for time-series smoothing.
    """
    return low_pass_filter(previous_value, new_value, alpha)


def complementary_filter(primary_value, secondary_value, alpha):
    """
    Complementary filter for combining two signals.
    Example: combining barometer (smooth) and ToF (accurate at low altitudes).
    alpha: weight for primary_value
    """
    return alpha * primary_value + (1 - alpha) * secondary_value


def smooth_derivative(previous_derivative, raw_derivative, alpha):
    """
    Smooth a derivative term using exponential filtering.
    Useful for PID derivative terms to reduce noise spikes.
    alpha: smoothing factor in [0, 1]
    """
    return alpha * raw_derivative + (1 - alpha) * previous_derivative