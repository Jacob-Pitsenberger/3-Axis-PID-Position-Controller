import time
from .state_estimator import StateEstimator
from control.pid import pid_x, pid_y, pid_z, pid_yaw
from utils.logger import DataLogger


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
        self.logger = DataLogger(filename="pid_flight.csv")

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

            # Started with value of 15 but this was too aggressive and testing logs showed it occasionally overpowering Tello's internal stabilization.
            vel_damping_gain = 10.0  # Reducing the value to 10 should minimize the 'fight' between my PID controller and the Tello's internal controller.

            # --- X/Y/Z PID corrections ---
            lr_cmd = pid_x.compute(self.setpoint["x"], est.position[0]) - vel_damping_gain * vx
            fb_cmd = pid_y.compute(self.setpoint["y"], est.position[1]) - vel_damping_gain * vy
            ud_cmd = pid_z.compute(self.setpoint["z"], est.position[2])

            # --- Yaw PID correction ---
            yaw_error = self._angle_difference(self.target_yaw, est.attitude[2])
            yaw_cmd = pid_yaw.compute(0.0, yaw_error)  # setpoint = 0 (no yaw error)

            # ---------------------------------------
            # 4. Log data (now includes loop_dt)
            # ---------------------------------------
            elapsed = time.time() - loop_start

            self.logger.log({
                "time": timestamp,
                "loop_dt": elapsed,

                # --- Estimated state (already logged) ---
                "x": est.position[0],
                "y": est.position[1],
                "z": est.position[2],
                "vx": est.velocity[0],
                "vy": est.velocity[1],
                "vz": est.velocity[2],
                "pitch": est.attitude[0],
                "roll": est.attitude[1],
                "yaw": est.attitude[2],

                # --- PID outputs ---
                "pid_x": lr_cmd,
                "pid_y": fb_cmd,
                "pid_z": ud_cmd,

                # --- RC commands sent to drone ---
                "rc_lr": lr_cmd,
                "rc_fb": fb_cmd,
                "rc_ud": ud_cmd,
                "rc_yaw": yaw_cmd,

                # --- RAW SENSOR VALUES (NEW) ---
                "tof_cm": raw_state.elevation[0],
                "barometer_m": raw_state.elevation[1],
                "height_cm": raw_state.elevation[2],

                "agx": raw_state.acceleration[0],
                "agy": raw_state.acceleration[1],
                "agz": raw_state.acceleration[2],

                "vel_raw_x": raw_state.velocity[0],
                "vel_raw_y": raw_state.velocity[1],
                "vel_raw_z": raw_state.velocity[2],

                "pitch_raw": raw_state.orientation[0],
                "roll_raw": raw_state.orientation[1],
                "yaw_raw": raw_state.orientation[2],

                "pid_yaw": yaw_cmd,

                "battery:": raw_state.battery
            })

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