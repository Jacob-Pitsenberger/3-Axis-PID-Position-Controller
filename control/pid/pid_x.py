"""
Preconfigured PID controller for X-axis position control.
Loads tuning parameters from config/pid_config.json.
"""

from .pid_base import PID, PIDConfig
from utils.config_loader import load_pid_config

# Load configuration for X-axis PID
_cfg = load_pid_config()["pid_x"]

pid_x = PID(PIDConfig(
    kp=_cfg["kp"],
    ki=_cfg["ki"],
    kd=_cfg["kd"],
    output_limits=tuple(_cfg["output_limits"]),
    integral_limits=tuple(_cfg["integral_limits"]),
    derivative_filter_alpha=_cfg["derivative_filter_alpha"]
))