import time
from dataclasses import dataclass
from drone.drone_state import DroneState
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
    angular_velocity: list  # [pitch_rate, roll_rate, yaw_rate] in deg/s


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

        # Altitude baselines (set on first estimate call)
        self._baseline_tof_m = None
        self._baseline_baro_m = None
        self._baseline_height_m = None

        # Storage for previous yaw and previous yaw-rate (for computing yaw rate numerically since Tello doesn't expose this).
        self.prev_yaw = None
        self.prev_yaw_rate = 0.0
        self.alpha_yaw_rate = 0.7  # low‑pass filter weight

    def _fuse_altitude(self, elevation):
        """
        Fuse ToF, barometer, and height into a stable altitude estimate.
        elevation = [tof_cm, baro_m, height_cm]

        Returns altitude in meters, relative to the baseline at takeoff.
        """
        tof_m = elevation[0] / 100.0
        baro_m = elevation[1]
        height_m = elevation[2] / 100.0

        # Initialize baselines at first call (assumed near ground / takeoff)
        if self._baseline_tof_m is None:
            self._baseline_tof_m = tof_m
            self._baseline_baro_m = baro_m
            self._baseline_height_m = height_m

        # Work in relative terms
        rel_tof = tof_m - self._baseline_tof_m
        rel_baro = baro_m - self._baseline_baro_m
        rel_height = height_m - self._baseline_height_m

        # ToF is accurate at low altitudes, height is stable, baro is smooth
        raw_z = 0.5 * rel_tof + 0.3 * rel_height + 0.2 * rel_baro

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

        # ----------------------------------------------------
        # Compute dt for integration and yaw-rate calculation
        # ----------------------------------------------------
        if self.prev_time is None:
            dt = 0.0
        else:
            dt = now - self.prev_time

        self.prev_time = now

        # ----------------------------------------------------
        # Extract attitude (pitch, roll, yaw)
        # ----------------------------------------------------
        pitch, roll, yaw = state.orientation

        # ----------------------------------------------------
        # Convert body-frame velocity → world-frame
        # ----------------------------------------------------
        vx_w, vy_w, vz_w = body_to_world_velocity(state.velocity, yaw)

        # ----------------------------------------------------
        # Integrate horizontal position (x, y)
        # ----------------------------------------------------
        if dt > 0:
            self.x += vx_w * dt
            self.y += vy_w * dt

        # ----------------------------------------------------
        # Fuse altitude sensors into stable z estimate
        # ----------------------------------------------------
        z = self._fuse_altitude(state.elevation)

        # ----------------------------------------------------
        # Compute yaw-rate numerically (Tello does not expose gyro)
        # ----------------------------------------------------
        if self.prev_yaw is None or dt == 0:
            yaw_rate = 0.0
        else:
            # Handle wrap-around (e.g., 179° → -179°)
            dyaw = yaw - self.prev_yaw
            if dyaw > 180:
                dyaw -= 360
            elif dyaw < -180:
                dyaw += 360

            # Numerical derivative
            yaw_rate = dyaw / dt

        # Low-pass filter yaw-rate for stability
        yaw_rate = (
                self.alpha_yaw_rate * yaw_rate +
                (1 - self.alpha_yaw_rate) * self.prev_yaw_rate
        )

        # Store for next iteration
        self.prev_yaw = yaw
        self.prev_yaw_rate = yaw_rate

        # ----------------------------------------------------
        # Build and return the estimated state object
        # (pitch_rate and roll_rate unavailable → set to 0.0)
        # ----------------------------------------------------
        est = EstimatedState(
            position=[self.x, self.y, z],
            velocity=[vx_w, vy_w, vz_w],
            attitude=[pitch, roll, yaw],
            angular_velocity=[0.0, 0.0, yaw_rate]  # Only yaw-rate is computed
        )

        return est

    def reset(self):
        """Reset estimator state, including altitude baselines."""
        self.x = 0.0
        self.y = 0.0
        self.prev_time = None
        self.prev_z = 0.0
        self._baseline_tof_m = None
        self._baseline_baro_m = None
        self._baseline_height_m = None