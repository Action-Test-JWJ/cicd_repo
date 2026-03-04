#include "edie_filtering_odometry/odom_utils.hpp"

OdomUtils::OdomUtils()
{
}

OdomUtils::~OdomUtils()
{
}

double OdomUtils::NormalizeAngleToPi(double z)
{
  return atan2(sin(z), cos(z));
}

double OdomUtils::GetAngleDiff(double a, double b)
{
  a = NormalizeAngleToPi(a);
  b = NormalizeAngleToPi(b);
  double d1 = a - b;
  double d2 = 2 * M_PI - fabs(d1);
  if (d1 > 0) {
    d2 *= -1.0;
  }
  if (fabs(d1) < fabs(d2)) {
    return d1;
  } else {
    return d2;
  }
}

size_t OdomUtils::FindMinErrorIdx(geometry_msgs::msg::PoseArray samples, double weight_pos, double weight_yaw)
{
    // 1) samples.poses 개수
    size_t N = samples.poses.size();

    // 2) 각 샘플의 총 에러 저장용
    std::vector<double> total_error(N, 0.0);

    // 3) 이중 루프 돌면서 pos_error, yaw_error 누적
    for (size_t i = 0; i < N; ++i) {
        tf2::Quaternion qi(
            samples.poses[i].orientation.x,
            samples.poses[i].orientation.y,
            samples.poses[i].orientation.z,
            samples.poses[i].orientation.w
        );
        double ri, pti, yawi;
        tf2::Matrix3x3(qi).getRPY(ri, pti, yawi);

        double pe_sum = 0.0, ye_sum = 0.0;
        for (size_t j = 0; j < N; ++j) {
            if (i == j) continue;

            // 위치 거리
            double dx = samples.poses[i].position.x - samples.poses[j].position.x;
            double dy = samples.poses[i].position.y - samples.poses[j].position.y;
            // 유클리드 거리 나타냄
            pe_sum += std::hypot(dx, dy);

            // 각도 차
            tf2::Quaternion qj(
                samples.poses[j].orientation.x,
                samples.poses[j].orientation.y,
                samples.poses[j].orientation.z,
                samples.poses[j].orientation.w
            );
            double rj, ptj, yawj;
            tf2::Matrix3x3(qj).getRPY(rj, ptj, yawj);
            ye_sum += std::fabs(GetAngleDiff(yawi, yawj));
        }

        pe_sum /= double(N - 1);    
        ye_sum /= double(N - 1);

        // 가중합
        total_error[i] = weight_pos * pe_sum
                        + weight_yaw * ye_sum;
    }

    // 4) 최소 에러 인덱스 찾기
    auto min_it = std::min_element(total_error.begin(), total_error.end());
    size_t rep_idx = std::distance(total_error.begin(), min_it);

    return rep_idx;
}

double OdomUtils::yawFromQuat(const geometry_msgs::msg::Quaternion& q){
  tf2::Quaternion tq(q.x,q.y,q.z,q.w); 
  double r,p,y; 
  tf2::Matrix3x3(tq).getRPY(r,p,y); 
  
  return y;
}

// geometry_msgs::msg::Pose 를 만들어 주는 헬퍼
geometry_msgs::msg::Pose OdomUtils::GetPose(
    double x, double y, double z, double yaw_rad)
{
    geometry_msgs::msg::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    p.orientation = tf2::toMsg(q);
    return p;
}

void OdomUtils::SetPubOdomInfo(
    const nav_msgs::msg::Odometry &in,
    nav_msgs::msg::Odometry &out, 
    const geometry_msgs::msg::Pose &pose_filtered,
    const std::string              &child_frame_id)
{
    out.header    = in.header;
    out.child_frame_id = child_frame_id;
    
    // 상태 값만 업데이트
    out.pose.pose = pose_filtered;
    // out.pose.covariance = in.pose.covariance;
    // // x, y, z, roll, pitch, yaw
    // out.pose.covariance[0] = 5e-2; // x
    // out.pose.covariance[7] = 5e-2; // y
    // out.pose.covariance[14] = 1e-9; // z
    // out.pose.covariance[21] = 1e-9; // roll
    // out.pose.covariance[28] = 1e-9; // pitch
    // out.pose.covariance[35] = 6e-2; // yaw

    out.twist.twist = in.twist.twist;
    // // vx, vy, vz, vroll, vpitch, vyaw
    // out.twist.covariance[0] = 5e-4; // vx
    // out.twist.covariance[7] = 1e-9; // vy
    // out.twist.covariance[14] = 1e-9; // vz
    // out.twist.covariance[21] = 1e-9; // vroll
    // out.twist.covariance[28] = 1e-9; // vpitch
    // out.twist.covariance[35] = 6.5e-4; // vyaw
}

void OdomUtils::SetPubPoseInfo(
    const nav_msgs::msg::Odometry &in,
    geometry_msgs::msg::PoseStamped &out,  
    const geometry_msgs::msg::Pose &pose_filtered)
{
    out.header    = in.header;
    out.pose = pose_filtered;
}