from drone.drone_interface import DroneInterface
from controller.controller import Controller


def main():
    """
    Entry point for the 3-axis PID position controller.
    Initializes the drone interface and starts the control loop.
    """
    drone = DroneInterface()

    # Target hover altitude in meters
    target_altitude = 0.5

    controller = Controller(drone_interface=drone, target_altitude=target_altitude)

    try:
        controller.start()
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        print("Shutting down controller...")
        controller.stop()


if __name__ == "__main__":
    main()