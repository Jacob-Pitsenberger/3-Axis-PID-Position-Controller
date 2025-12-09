"""
Utility package providing reusable components for filtering,
logging, and coordinate transforms used throughout the drone
control and state‑estimation pipeline.

This package exposes:
- Filtering utilities (low‑pass, EMA, complementary filters)
- Lightweight CSV logging tools
- Frame‑transform helpers for converting body-frame velocities
  into world-frame velocities
"""

# Re-export filtering utilities
from .filters import (
    low_pass_filter,
    exponential_moving_average,
    complementary_filter,
    smooth_derivative,
)

# Re-export logger class
from .logger import DataLogger

# Re-export transform utilities
from .transforms import body_to_world_velocity

__all__ = [
    "low_pass_filter",
    "exponential_moving_average",
    "complementary_filter",
    "smooth_derivative",
    "DataLogger",
    "body_to_world_velocity",
]