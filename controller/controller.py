import time
from .state_estimator import StateEstimator
from control.pid import pid_x, pid_y, pid_z, pid_yaw
from utils.logger import DataLogger
from utils.log_context import LogContext, PIDOutputs, RCOutputs


class Controller:
    """
    Main control loop for 3-axis PID position hold.
    Orchestrates:
    - Sensor updates
    - State estimation
    - PID corrections
    - RC command output
    - Loop timing and fail safes
    """

    def __init__(self, drone_interface, target_altitude=0.5):
        """
        drone_interface: object implementing connect(), takeoff(), land(),
                         get_state(), send_rc(lr, fb, ud, yaw)
        target_altitude: desired hover altitude in meters
        """
        self.drone = drone_interface
        self.state_estimator = StateEstimator()

        # Logger now handles frequency internally
        self.logger = DataLogger(
            filename="pid_flight.csv",
            mode="full_pid",
            log_every_n=5   # ~5 Hz logging at 25 Hz loop
        )

        # Desired hover point (x=0, y=0, z=target_altitude)
        self.setpoint = {
            "x": 0.0,
            "y": 0.0,
            "z": target_altitude
        }

        # Loop timing
        self.loop_rate_hz = 25.0
        self.loop_dt = 1.0 / self.loop_rate_hz

        # Failsafe flag
        self.running = True

        # Initialize yaw target (set properly after takeoff)
        self.target_yaw = None

    def start(self):
        """Initialize drone and begin control loop."""
        print("Connecting to drone...")
        self.drone.connect()

        print("Taking off...")
        self.drone.takeoff()

        print("Stabilizing before starting PID loop...")
        time.sleep(2.0)

        # Reset estimator AFTER takeoff and stabilization
        self.state_estimator.reset()

        # Lock yaw heading at takeoff
        initial_state = self.drone.get_state()
        self.target_yaw = initial_state.orientation[2]

        print("estimator reset... Starting PID loop...")

        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("Kill switch activated.")
        finally:
            print("Exiting...")
            print("Landing...")
            self.drone.land()
            self.logger.close()

    @staticmethod
    def _angle_difference(target, current):
        """
        Compute the shortest signed angular difference between two yaw angles (degrees).
        Ensures result is in [-180, 180].
        """
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

    def control_loop(self):
        """Main PID control loop running at ~25 Hz."""
        print("Starting control loop...")

        while self.running:
            loop_start = time.time()
            timestamp = loop_start  # unified timestamp for this iteration

            # ---------------------------------------
            # 1. Pull raw sensor data
            # ---------------------------------------
            raw_state = self.drone.get_state()  # returns DroneState

            # ---------------------------------------
            # 2. Estimate world-frame state
            # ---------------------------------------
            est = self.state_estimator.estimate(raw_state)

            # ---------------------------------------
            # 3. Compute PID corrections
            # ---------------------------------------

            # Velocity damping (feedforward)
            vx = est.velocity[0]
            vy = est.velocity[1]

            vel_damping_gain = 8.0

            # --- X/Y/Z PID corrections ---
            lr_cmd = pid_x.compute(self.setpoint["x"], est.position[0]) - vel_damping_gain * vx
            fb_cmd = pid_y.compute(self.setpoint["y"], est.position[1]) - vel_damping_gain * vy
            ud_cmd = pid_z.compute(self.setpoint["z"], est.position[2])

            # --- Yaw PID correction ---
            yaw_error = self._angle_difference(self.target_yaw, est.attitude[2])
            yaw_cmd = pid_yaw.compute(0.0, yaw_error)

            # Yaw-rate damping
            yaw_rate = est.angular_velocity[2]
            yaw_damping_gain = 0.7
            yaw_cmd -= yaw_damping_gain * yaw_rate

            # ---------------------------------------
            # 4. Build logging context and log frame
            # ---------------------------------------
            elapsed = time.time() - loop_start

            ctx = LogContext(
                timestamp=timestamp,
                loop_dt=elapsed,
                est=est,
                raw=raw_state,
                pid=PIDOutputs(lr_cmd, fb_cmd, ud_cmd, yaw_cmd),
                rc=RCOutputs(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)
            )

            self.logger.log_frame(ctx)

            # ---------------------------------------
            # 5. Send RC command to drone
            # ---------------------------------------
            self.drone.send_rc(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

            # ---------------------------------------
            # 6. Maintain loop timing
            # ---------------------------------------
            sleep_time = self.loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        """External stop trigger for fail safes."""
        self.running = False
        self.logger.close()