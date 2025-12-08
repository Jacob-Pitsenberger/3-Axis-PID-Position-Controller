import math

def body_to_world_velocity(velocity, yaw_deg):
    """
    Convert body-frame velocities (vgx, vgy, vgz) into world-frame velocities.
    velocity: [vgx, vgy, vgz] in dm/s
    yaw_deg: yaw angle in degrees
    Returns:
        vx_w, vy_w, vz_w in m/s
    """
    # Convert dm/s → m/s
    vx_b = velocity[0] / 100.0
    vy_b = velocity[1] / 100.0
    vz_b = velocity[2] / 100.0

    yaw_rad = math.radians(yaw_deg)

    # 2D rotation around Z-axis (yaw)
    vx_w = vx_b * math.cos(yaw_rad) - vy_b * math.sin(yaw_rad)
    vy_w = vx_b * math.sin(yaw_rad) + vy_b * math.cos(yaw_rad)

    return vx_w, vy_w, vz_b