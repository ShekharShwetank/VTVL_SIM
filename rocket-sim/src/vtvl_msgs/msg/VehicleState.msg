# VehicleState.msg: rocket state vector (positions, velocities, attitude, rates)

# Header to track timestamp and frame ID
std_msgs/Header header

# Translational State in Inertial Frame
geometry_msgs/Point position      # Inertial position (x, y, z) in meters
geometry_msgs/Vector3 velocity  # Inertial velocity (vx, vy, vz) in m/s

# Rotational State
geometry_msgs/Quaternion attitude # Orientation as a quaternion (x, y, z, w)
geometry_msgs/Vector3 angular_velocity # Body frame angular rates (wx, wy, wz) in rad/s

# Mass Properties
float64 mass # Current total mass in kg
