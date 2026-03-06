std_msgs/Header header  # ROS 메시지 헤더 (stamp와 frame_id 포함)

EndpointState[] endpoints  # 엔드포인트 배열 (중심, 왼손, 오른손, 왼발, 오른발, 머리 포함)

# CoM (Center of Mass)
geometry_msgs/Point com_position  # CoM 위치 (x_com, y_com, z_com)
geometry_msgs/Vector3 com_velocity  # CoM 속도 (vx_com, vy_com, vz_com)
geometry_msgs/Vector3 com_acceleration  # CoM 가속도 (ax_com, ay_com, az_com)

geometry_msgs/Point dcm_position  # CoM 위치 (x_dcm, y_dcm, z_dcm)

# ZMP (Zero Moment Point)
geometry_msgs/Point zmp_position  # ZMP 위치 (x_zmp, y_zmp)

geometry_msgs/Vector3 angular_momentum

geometry_msgs/Point cop_position
geometry_msgs/Point cop_position_fsr              # (월드)
geometry_msgs/Point left_cop_position_local
geometry_msgs/Point right_cop_position_local
bool left_foot_contact
bool right_foot_contact

## Gait Phase
int16 lifting_state

int16 K_AIR_STANCE          = 0
int16 K_RIGHT_SINGLE_STANCE = 1
int16 K_LEFT_SINGLE_STANCE  = 2
int16 K_DOUBLE_STANCE       = 3
# 센서 데이터
Sensors sensors  # IMU, FSR, 액추에이터 데이터를 포함한 원시 데이터
