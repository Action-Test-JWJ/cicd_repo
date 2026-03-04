#include "edie_localization_manager/utility/odom_tools.hpp"
#include <iostream>

namespace odom_tools
{
    // odom을 이용해 pose_with_info_stamped를 구성
    edie_msgs::msg::PoseWithInfoStamped CreatePoseInfoFromOdom(const nav_msgs::msg::Odometry& odom)
    {
        // Odometry 데이터에서 x, y, theta 정보 추출
        edie_msgs::msg::PoseWithInfoStamped pose_data;
        pose_data.pose.x = odom.pose.pose.position.x;
        pose_data.pose.y = odom.pose.pose.position.y;

        // 쿼터니언에서 오일러 각도로 변환하여 theta 값 추출
        double qx = odom.pose.pose.orientation.x;
        double qy = odom.pose.pose.orientation.y;
        double qz = odom.pose.pose.orientation.z;
        double qw = odom.pose.pose.orientation.w;
        // 쿼터니언을 yaw(theta)로 변환
        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        double theta = std::atan2(siny_cosp, cosy_cosp);
        // 각도 범위를 0 ~ 2π로 조정
        if (theta < 0.0)
        {
            theta += 2.0 * M_PI;
        }
        pose_data.pose.theta = theta;

        pose_data.header = odom.header;
        // pose_data.timestamp = std::chrono::steady_clock::now();

        // key-value에 type과 odometry 정보 추가
        diagnostic_msgs::msg::KeyValue info_type;
        info_type.key = "type";
        info_type.value = "odometry";
        pose_data.info.push_back(info_type);

        return pose_data;
    }

    // pose2d와 robot_base를 이용해 odom을 구성
    nav_msgs::msg::Odometry CreateOdomFromPose(const edie_msgs::msg::PoseWithInfoStamped& pose, const geometry_msgs::msg::Pose& robot_base)
    {
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = pose.pose.x;
        odom.pose.pose.position.y = pose.pose.y;
        odom.pose.pose.position.z = robot_base.position.z; // z값은 robot_base에서 가져옴

        // robot_base에서 roll, pitch를 추출하고, pose에서 yaw를 사용하여 쿼터니언 계산
        // 먼저 robot_base 쿼터니언에서 roll과 pitch 추출
        double qx = robot_base.orientation.x;
        double qy = robot_base.orientation.y;
        double qz = robot_base.orientation.z;
        double qw = robot_base.orientation.w;

        // 쿼터니언에서 오일러 각도(roll, pitch) 추출
        double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        double roll = std::atan2(sinr_cosp, cosr_cosp);
        // roll 각도 범위를 0~2π로 조정
        if (roll < 0.0)
        {
            roll += 2.0 * M_PI;
        }

        double sinp = 2.0 * (qw * qy - qz * qx);
        double pitch;
        if (std::abs(sinp) >= 1)
            pitch = std::copysign(M_PI / 2, sinp); // 우세현상 방지
        else
            pitch = std::asin(sinp);
        // pitch 각도 범위를 0~2π로 조정
        if (pitch < 0.0)
        {
            pitch += 2.0 * M_PI;
        }

        // pose의 theta에서 yaw 가져옴
        double yaw = pose.pose.theta;

        // roll, pitch, yaw를 새로운 쿼터니언으로 변환
        // 출처: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        double cp = std::cos(pitch * 0.5);
        double sp = std::sin(pitch * 0.5);
        double cr = std::cos(roll * 0.5);
        double sr = std::sin(roll * 0.5);

        odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy;
        odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy;
        odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy;
        odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy;

        return odom;
    }

    // localization_odom을 이용해 odom_to_pelvis를 구성
    nav_msgs::msg::Odometry TransformOdomToPelvis(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Odometry& localization_odom)
    {
        nav_msgs::msg::Odometry odom_to_pelvis;

        // 위치 차이 계산
        odom_to_pelvis.pose.pose.position.x = odom.pose.pose.position.x - localization_odom.pose.pose.position.x;
        odom_to_pelvis.pose.pose.position.y = odom.pose.pose.position.y - localization_odom.pose.pose.position.y;
        odom_to_pelvis.pose.pose.position.z = odom.pose.pose.position.z - localization_odom.pose.pose.position.z;

        // 회전 설정
        odom_to_pelvis.pose.pose.orientation.x = 0.0;
        odom_to_pelvis.pose.pose.orientation.y = 0.0;
        odom_to_pelvis.pose.pose.orientation.z = std::sin(odom.pose.pose.orientation.z / 2.0);
        odom_to_pelvis.pose.pose.orientation.w = std::cos(odom.pose.pose.orientation.z / 2.0);

        // 타임스탬프 복사
        odom_to_pelvis.header = odom.header;

        return odom_to_pelvis;
    }

}
