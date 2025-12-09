"""
High-level control orchestration package.

This package includes:
- Controller: the main PID control loop for 3-axis position hold
- StateEstimator: sensor fusion logic that converts raw drone telemetry
  into world-frame position, velocity, and attitude estimates

These components coordinate sensor updates, PID corrections, RC command
output, and logging during flight.
"""

from .controller import Controller
from .state_estimator import StateEstimator

__all__ = [
    "Controller",
    "StateEstimator"
]