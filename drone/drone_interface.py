from djitellopy import Tello
from .drone_state import DroneState


class DroneInterface:
    """
    Hardware abstraction layer for the Tello drone.
    Provides a clean, consistent API for the controller.
    """

    def __init__(self):
        self.drone = Tello()
        self.connected = False

    # ---------------------------------------------------------
    # Connection & Flight Control
    # ---------------------------------------------------------

    def connect(self):
        """Connect to the Tello drone and initialize the SDK."""
        self.drone.connect()
        self.connected = True
        print(f"Battery: {self.drone.get_battery()}%")

    def takeoff(self):
        """Command the drone to take off."""
        if not self.connected:
            raise RuntimeError("Drone not connected.")
        self.drone.takeoff()

    def land(self):
        """Command the drone to land."""
        if not self.connected:
            return
        self.drone.land()

    # ---------------------------------------------------------
    # Sensor State Retrieval
    # ---------------------------------------------------------

    def get_state(self) -> DroneState:
        """
        Query Tello telemetry and return a DroneState object.
        This keeps the rest of the system drone-agnostic.
        """
        return DroneState(
            orientation=[
                self.drone.get_pitch(),
                self.drone.get_roll(),
                self.drone.get_yaw()
            ],
            velocity=[
                self.drone.get_speed_x(),
                self.drone.get_speed_y(),
                self.drone.get_speed_z()
            ],
            elevation=[
                self.drone.get_distance_tof(),  # cm
                self.drone.get_barometer(),     # m
                self.drone.get_height()         # cm
            ],
            acceleration=[
                self.drone.get_acceleration_x(),
                self.drone.get_acceleration_y(),
                self.drone.get_acceleration_z()
            ],
            battery = self.drone.get_battery()
        )

    # ---------------------------------------------------------
    # RC Command Output
    # ---------------------------------------------------------

    def send_rc(self, lr, fb, ud, yaw):
        """
        Send RC control commands to the drone.
        Values should be in the range [-100, 100].
        """
        self.drone.send_rc_control(int(lr), int(fb), int(ud), int(yaw))