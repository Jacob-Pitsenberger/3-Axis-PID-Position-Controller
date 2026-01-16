FULL_PID_PROFILE = {
    "time": lambda c: c.timestamp,
    "loop_dt": lambda c: c.loop_dt,

    # Estimated state
    "x": lambda c: c.est.position[0],
    "y": lambda c: c.est.position[1],
    "z": lambda c: c.est.position[2],
    "vx": lambda c: c.est.velocity[0],
    "vy": lambda c: c.est.velocity[1],
    "vz": lambda c: c.est.velocity[2],
    "pitch": lambda c: c.est.attitude[0],
    "roll": lambda c: c.est.attitude[1],
    "yaw": lambda c: c.est.attitude[2],
    "yaw_rate": lambda c: c.est.angular_velocity[2],

    # PID outputs
    "pid_x": lambda c: c.pid.lr,
    "pid_y": lambda c: c.pid.fb,
    "pid_z": lambda c: c.pid.ud,
    "pid_yaw": lambda c: c.pid.yaw,

    # RC outputs
    "rc_lr": lambda c: c.rc.lr,
    "rc_fb": lambda c: c.rc.fb,
    "rc_ud": lambda c: c.rc.ud,
    "rc_yaw": lambda c: c.rc.yaw,

    # Raw state
    "tof_cm": lambda c: c.raw.elevation[0],
    "barometer_m": lambda c: c.raw.elevation[1],
    "height_cm": lambda c: c.raw.elevation[2],
    "agx": lambda c: c.raw.acceleration[0],
    "agy": lambda c: c.raw.acceleration[1],
    "agz": lambda c: c.raw.acceleration[2],
    "vel_raw_x": lambda c: c.raw.velocity[0],
    "vel_raw_y": lambda c: c.raw.velocity[1],
    "vel_raw_z": lambda c: c.raw.velocity[2],
    "pitch_raw": lambda c: c.raw.orientation[0],
    "roll_raw": lambda c: c.raw.orientation[1],
    "yaw_raw": lambda c: c.raw.orientation[2],
    "battery": lambda c: c.raw.battery,
}

SUPERVISORY_PROFILE = {
    "time": lambda c: c.timestamp,
    "loop_dt": lambda c: c.loop_dt,
    "x": lambda c: c.est.position[0],
    "y": lambda c: c.est.position[1],
    "z": lambda c: c.est.position[2],
    "vx": lambda c: c.est.velocity[0],
    "vy": lambda c: c.est.velocity[1],
    "vz": lambda c: c.est.velocity[2],
    "yaw": lambda c: c.est.attitude[2],
    "rc_lr": lambda c: c.rc.lr,
    "rc_fb": lambda c: c.rc.fb,
    "rc_ud": lambda c: c.rc.ud,
    "rc_yaw": lambda c: c.rc.yaw,
    "battery": lambda c: c.raw.battery,
}