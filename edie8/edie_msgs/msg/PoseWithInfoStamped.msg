# PoseWithInfoStamped.msg
std_msgs/Header header                       # timestamp
geometry_msgs/Pose2D pose                    # x, y, θ
float32 delta_time                           # 카메라로부터 결과 도출까지의 시간
diagnostic_msgs/KeyValue[] info     # 각 localization 방법별 특징 값