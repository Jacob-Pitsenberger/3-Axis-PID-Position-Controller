from dataclasses import dataclass

@dataclass
class DroneState:
    orientation: list   # [pitch, roll, yaw]
    velocity: list      # [vgx, vgy, vgz]
    elevation: list     # [tof, baro, height]
    acceleration: list  # [agx, agy, agz]