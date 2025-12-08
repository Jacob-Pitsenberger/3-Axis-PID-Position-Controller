from dataclasses import dataclass

@dataclass
class DroneState:
    orientation: list   # [pitch, roll, yaw]
    velocity: list      # [vgx, vgy, vgz]
    elevation: list     # [tof, baro, height]
    acceleration: list  # [agx, agy, agz]

    def update_from_drone(self, drone):
        """
        Pull fresh sensor data from the drone and update this state object.
        """
        self.orientation = [
            drone.get_pitch(),
            drone.get_roll(),
            drone.get_yaw()
        ]

        self.velocity = [
            drone.get_speed_x(),
            drone.get_speed_y(),
            drone.get_speed_z()
        ]

        self.elevation = [
            drone.get_distance_tof(),  # cm
            drone.get_barometer(),     # m
            drone.get_height()         # cm
        ]

        self.acceleration = [
            drone.get_acceleration_x(),
            drone.get_acceleration_y(),
            drone.get_acceleration_z()
        ]