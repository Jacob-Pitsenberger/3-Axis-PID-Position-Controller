"""
Drone hardware abstraction package.

This package provides:
- DroneInterface: a high-level wrapper around the Tello SDK,
  offering a clean, consistent API for connection, flight
  commands, and telemetry retrieval.
- DroneState: a structured dataclass encapsulating all raw
  sensor readings returned by the drone.

These modules allow the rest of the control system to remain
drone‑agnostic and modular.
"""

from .drone_interface import DroneInterface
from .drone_state import DroneState

__all__ = [
    "DroneInterface",
    "DroneState",
]