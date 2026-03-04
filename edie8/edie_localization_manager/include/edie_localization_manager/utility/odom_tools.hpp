#ifndef ODOM_TOOLS_HPP_
#define ODOM_TOOLS_HPP_

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "edie_msgs/msg/pose_with_info_stamped.hpp"
#include <cmath> // 수학 함수를 위한 헤더 추가

namespace odom_tools
{
    // odom을 이용해 pose_with_info_stamped를 구성
    edie_msgs::msg::PoseWithInfoStamped CreatePoseInfoFromOdom(const nav_msgs::msg::Odometry& odom);

    // pose와 robot_base를 이용해 odom을 구성
    nav_msgs::msg::Odometry CreateOdomFromPose(const edie_msgs::msg::PoseWithInfoStamped& pose, const geometry_msgs::msg::Pose& robot_base);

    // localization_odom을 이용해 odom_to_pelvis를 구성
    nav_msgs::msg::Odometry TransformOdomToPelvis(const nav_msgs::msg::Odometry& odom, const nav_msgs::msg::Odometry& localization_odom);

}

#endif
