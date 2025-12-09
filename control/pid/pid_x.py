"""
Preconfigured PID controller for X-axis position control.
"""

from .pid_base import PID, PIDConfig

pid_x = PID(PIDConfig(
    kp=40.0,
    ki=0.0,
    kd=20.0,
    output_limits=(-100, 100),
    integral_limits=(-30, 30),
    derivative_filter_alpha=0.6
))