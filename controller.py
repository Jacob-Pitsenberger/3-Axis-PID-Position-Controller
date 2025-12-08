import time
from drone_state import DroneState
from state_estimator import StateEstimator
from pid import pid_x, pid_y, pid_z


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

    def start(self):
        """Initialize drone and begin control loop."""
        print("Connecting to drone...")
        self.drone.connect()

        print("Taking off...")
        self.drone.takeoff()

        print("Stabilizing before starting PID loop...")
        time.sleep(2.0)

        try:
            self.control_loop()
        except KeyboardInterrupt:
            print("Kill switch activated.")
        finally:
            print("Landing...")
            self.drone.land()

    def control_loop(self):
        """Main PID control loop running at ~25 Hz."""
        print("Starting control loop...")

        while self.running:
            loop_start = time.time()

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
            # X-axis (left/right)
            lr_cmd = pid_x.compute(self.setpoint["x"], est.position[0])

            # Y-axis (forward/back)
            fb_cmd = pid_y.compute(self.setpoint["y"], est.position[1])

            # Z-axis (up/down)
            ud_cmd = pid_z.compute(self.setpoint["z"], est.position[2])

            # No yaw control for now (0)
            yaw_cmd = 0

            # ---------------------------------------
            # 4. Send RC command to drone
            # ---------------------------------------
            self.drone.send_rc(lr_cmd, fb_cmd, ud_cmd, yaw_cmd)

            # ---------------------------------------
            # 5. Maintain loop timing
            # ---------------------------------------
            elapsed = time.time() - loop_start
            sleep_time = self.loop_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        """External stop trigger for fail safes."""
        self.running = False