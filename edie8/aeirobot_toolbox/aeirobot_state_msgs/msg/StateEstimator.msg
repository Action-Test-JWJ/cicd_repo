# base
geometry_msgs/Point base_position
geometry_msgs/Vector3 base_velocity
geometry_msgs/Quaternion base_orientation

# foot 
geometry_msgs/Point left_foot_position
geometry_msgs/Point right_foot_position

# sensor bias
geometry_msgs/Vector3 bias_acc
geometry_msgs/Vector3 bias_gyro 

# covariance matrix 23x23
float64[] covariance 