std_msgs/Header header  # ROS 메시지 헤더 (stamp와 frame_id 포함)
int16 sensor_type
int16 K_FOOT = 1 # 이족보행용
int16 K_BODY = 2 # 피부촉감용
std_msgs/Float32MultiArray[] force
std_msgs/Float32MultiArray[] net_force