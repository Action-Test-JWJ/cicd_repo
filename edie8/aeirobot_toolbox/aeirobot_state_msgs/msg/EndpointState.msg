# 엔드포인트의 상태
string endpoint_name
geometry_msgs/Pose pose             # 위치 및 자세 (x, y, z, roll, pitch, yaw)
geometry_msgs/Pose local_pose       # 로컬 위치 및 자세 (x, y, z, roll, pitch, yaw)
geometry_msgs/Vector3 euler_angle
geometry_msgs/Vector3 local_euler_angle
geometry_msgs/Twist twist           # 선속도, 각속도
geometry_msgs/Twist local_twist     # 로컬 선속도, 각속도
geometry_msgs/Wrench wrench         # 힘, 돌림힘
geometry_msgs/Wrench external_wrench
