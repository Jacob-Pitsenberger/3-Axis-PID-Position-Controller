import time
from dataclasses import dataclass
from drone_state import DroneState
from utils.transforms import body_to_world_velocity

@dataclass
class EstimatedState:
    """
    High-level estimated state used by the PID controller.
    All values are in world-frame coordinates.
    """
    position: list     # [x, y, z] in meters
    velocity: list     # [vx, vy, vz] in m/s
    attitude: list     # [pitch, roll, yaw] in degrees


class StateEstimator:
    """
    Converts raw DroneState sensor readings into a filtered, world-frame
    position and velocity estimate suitable for PID control.
    """

    def __init__(self):
        # Integrated horizontal position (x, y)
        self.x = 0.0
        self.y = 0.0

        # Previous timestamp for integration
        self.prev_time = None

        # Altitude filtering parameters
        self.alpha_altitude = 0.7  # complementary filter weight

        # Previous fused altitude
        self.prev_z = 0.0

    def _fuse_altitude(self, elevation):
        """
        Fuse ToF, barometer, and height into a stable altitude estimate.
        elevation = [tof_cm, baro_m, height_cm]
        """
        tof_m = elevation[0] / 100.0
        baro_m = elevation[1]
        height_m = elevation[2] / 100.0

        # ToF is accurate at low altitudes, height is stable, baro is smooth
        raw_z = 0.5 * tof_m + 0.3 * height_m + 0.2 * baro_m

        # Complementary filter for smoothing
        if self.prev_time is None:
            self.prev_z = raw_z
        else:
            self.prev_z = (
                self.alpha_altitude * raw_z +
                (1 - self.alpha_altitude) * self.prev_z
            )

        return self.prev_z

    def estimate(self, state: DroneState) -> EstimatedState:
        """
        Main estimation function.
        Takes raw DroneState and returns an EstimatedState.
        """
        now = time.time()

        # Compute dt for integration
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time

        self.prev_time = now

        # Extract attitude
        pitch, roll, yaw = state.orientation

        # Convert body-frame velocity → world-frame
        vx_w, vy_w, vz_w = body_to_world_velocity(state.velocity, yaw)

        # Integrate horizontal position
        if dt > 0:
            self.x += vx_w * dt
            self.y += vy_w * dt

        # Fuse altitude sensors
        z = self._fuse_altitude(state.elevation)

        # Build estimated state object
        est = EstimatedState(
            position=[self.x, self.y, z],
            velocity=[vx_w, vy_w, vz_w],
            attitude=[pitch, roll, yaw]
        )

        return est