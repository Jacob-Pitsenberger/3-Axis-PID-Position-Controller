from djitellopy import tello


def main():
	# Connect to the drone and start receiving video
	drone = tello.Tello()
	drone.connect()
	drone.takeoff()
	# while True:
		# run PID RC control sending until the user issues a key press to land the drone.

	drone.land()

if __name__ == "__main__":
	main()