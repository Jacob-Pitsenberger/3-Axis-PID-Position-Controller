from dataclasses import dataclass
from controller.state_estimator import EstimatedState
from drone.drone_state import DroneState

@dataclass
class PIDOutputs:
    lr: float
    fb: float
    ud: float
    yaw: float

@dataclass
class RCOutputs:
    lr: float
    fb: float
    ud: float
    yaw: float

@dataclass
class LogContext:
    timestamp: float
    loop_dt: float
    est: EstimatedState
    raw: DroneState
    pid: PIDOutputs
    rc: RCOutputs