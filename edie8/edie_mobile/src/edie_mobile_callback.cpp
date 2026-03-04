#include "edie_mobile/edie_mobile_main.hpp"

void EdieMobileNode::RobotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    cur_pose_2D.x = msg->pose.position.x;
    cur_pose_2D.y = msg->pose.position.y;

    tf2::Quaternion quat(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    double yaw = tf2::getYaw(quat);

    cur_pose_2D.theta = yaw;
}

void EdieMobileNode::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // geometry_msgs::msg::Pose pose_msg;
    // pose_msg = msg->pose.pose;

    // pub_odom_to_pose->publish(pose_msg);

    // geometry_msgs::msg::PoseStamped pose_msg;
    // pose_msg.header = msg->header;
    // pose_msg.pose = msg->pose.pose;
    // pub_odom_to_pose->publish(pose_msg);

}

void EdieMobileNode::WayPointNumberCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    goal_point = msg->data;
    point_move_flag = true;
    // ROS_INFO("%d",goal_point);
    std::cout << goal_point << std::endl;
}

void EdieMobileNode::GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    goal_input_flag = true;

    goal_pose_2D.x = msg->pose.position.x;
    goal_pose_2D.y = msg->pose.position.y;
    goal_pose_2D.theta = tf2::getYaw(msg->pose.orientation);  // 쿼터니언 → yaw

    // RCLCPP_INFO(this->get_logger(), "Received goal pose: x=%.2f, y=%.2f, theta=%.2f",
    //             goal_pose_2D.x, goal_pose_2D.y, goal_pose_2D.theta);
}

void EdieMobileNode::OperationModeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
{
    operation_mode = msg->data;
}

void EdieMobileNode::DirectVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_direct_vel_ = *msg;
  direct_vel_active_ = true;
  goal_input_flag = false; //직접 제어 명령이 들어오면 경로 추종 목표를 취소
}