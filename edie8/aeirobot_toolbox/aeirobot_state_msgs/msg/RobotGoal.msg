std_msgs/Header header

sensor_msgs/JointState actuators
EndpointState root
EndpointState left_foot
EndpointState right_foot
geometry_msgs/Point zmp # (z=0)

uint8 balance_phase_id
float32 balance_phase_s

geometry_msgs/Vector3 left_foot_angle_delta
geometry_msgs/Vector3 right_foot_angle_delta
geometry_msgs/Vector3 root_angle_delta
geometry_msgs/Point root_position_delta
