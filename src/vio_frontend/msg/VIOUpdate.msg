std_msgs/Header header
vio_frontend/Feature[] features

# IMU Preintegration measurements
geometry_msgs/Vector3 delta_p
geometry_msgs/Vector3 delta_v
geometry_msgs/Quaternion delta_q
float64 dt

# Biases used for preintegration
geometry_msgs/Vector3 accel_bias
geometry_msgs/Vector3 gyro_bias
