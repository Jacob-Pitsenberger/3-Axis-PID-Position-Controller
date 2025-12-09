"""
Preconfigured PID controller for Z-axis (altitude) control.
"""

from .pid_base import PID, PIDConfig

pid_z = PID(PIDConfig(
    kp=60.0,
    ki=0.0,
    kd=30.0,
    output_limits=(-100, 100),
    integral_limits=(-20, 20),
    derivative_filter_alpha=0.7
))