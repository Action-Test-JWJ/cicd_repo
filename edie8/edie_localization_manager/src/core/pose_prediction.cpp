#include "edie_localization_manager/core/pose_prediction.hpp"

namespace pose_prediction
{

edie_msgs::msg::PoseWithInfoStamped UpdatePoseWithPrediction(
    bool odom_initialized,
    const std::deque<edie_msgs::msg::PoseWithInfoStamped>& pose_history,
    const tf2::Transform& odom_delta_sum,
    const rclcpp::Clock::SharedPtr& clock)
{
    edie_msgs::msg::PoseWithInfoStamped predicted_pose;
    if (!odom_initialized || pose_history.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("pose_prediction"), "NO Pose History or Odometry not initialized");
        return predicted_pose;
    }

    // 마지막으로 퓨전된 위치 가져오기
    geometry_msgs::msg::Pose2D last_fused_pose = pose_history.front().pose;

    // odometry 변화량을 기반으로 현재 위치 예측
    geometry_msgs::msg::Pose2D predicted_pose_2d = PredictPoseFromOdometry(last_fused_pose, odom_delta_sum);

    // 예측된 위치 설정
    predicted_pose.header.stamp = clock->now();
    predicted_pose.header.frame_id = "odom"; // or a common frame
    predicted_pose.pose = predicted_pose_2d;

    return predicted_pose;
}

geometry_msgs::msg::Pose2D PredictPoseFromOdometry(
    const geometry_msgs::msg::Pose2D& last_fused_pose,
    const tf2::Transform& odom_delta_sum)
{
    tf2::Transform fused_tf = Pose2DToTF(last_fused_pose);
    tf2::Transform predicted_tf = fused_tf * odom_delta_sum;
    return TFToPose2D(predicted_tf);
}

tf2::Transform OdometryMsgToTF(const nav_msgs::msg::Odometry& odom_msg)
{
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z));
    tf.setRotation(tf2::Quaternion(
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w));
    return tf;
}

tf2::Transform Pose2DToTF(const geometry_msgs::msg::Pose2D& pose)
{
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(pose.x, pose.y, 0.0));
    tf.setRotation(tf2::Quaternion(0, 0, std::sin(pose.theta / 2), std::cos(pose.theta / 2)));
    return tf;
}

geometry_msgs::msg::Pose2D TFToPose2D(const tf2::Transform& tf)
{
    geometry_msgs::msg::Pose2D pose;
    pose.x = tf.getOrigin().x();
    pose.y = tf.getOrigin().y();
    pose.theta = tf2::getYaw(tf.getRotation());
    return pose;
}

tf2::Transform ComputeOdomDelta(const tf2::Transform& prev_odom_tf,
                                 const tf2::Transform& curr_odom_tf)
{
    return prev_odom_tf.inverseTimes(curr_odom_tf);
}

} // namespace pose_prediction
