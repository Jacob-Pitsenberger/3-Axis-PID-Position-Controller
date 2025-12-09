"""
Control package exposing PID controllers for modular axis-specific control.

This package provides:
- pid_x, pid_y, pid_z: preconfigured PID controllers for X, Y, and Z axes
- PID, PIDConfig: reusable base classes for custom control logic
"""

from .pid.pid_x import pid_x
from .pid.pid_y import pid_y
from .pid.pid_z import pid_z
from .pid.pid_base import PID, PIDConfig

__all__ = [
    "pid_x",
    "pid_y",
    "pid_z",
    "PID",
    "PIDConfig"
]