#include "edie_filtering_odometry/edie_republishing_odom_node.hpp"

EdieRepublishingOdomNode::EdieRepublishingOdomNode()
    : Node("edie_republishing_odom_node"),
      is_first_odom_(true),
      tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
{
    rclcpp::QoS sub_qos(rclcpp::KeepLast(10)); // 1000 -> 10
    sub_qos.best_effort(); // 신뢰성 있는 QoS로 설정 (<-> RELIABLE)
    sub_qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // pub_raw_odom 퍼블리셔는 TRANSIENT_LOCAL durability를 적용하기 위한 별도 QoS 사용
    rclcpp::QoS raw_pub_qos(rclcpp::KeepLast(10));
    raw_pub_qos.reliable(); 
    raw_pub_qos.durability(rclcpp::DurabilityPolicy::Volatile); // 실시간 데이터만 필요하고, 과거 건 필요 없음 (<-> TRANSIENT_LOCAL)

    // Subscriber
    sub_raw_odom = create_subscription<nav_msgs::msg::Odometry>("/edie8/diff_drive_controller/odom", sub_qos,
        std::bind(&EdieRepublishingOdomNode::OdometryCallback, this, std::placeholders::_1));

    // Publisher
    pub_raw_odom = create_publisher<nav_msgs::msg::Odometry>("/edie8/localization/raw_odom", raw_pub_qos);
}

EdieRepublishingOdomNode::~EdieRepublishingOdomNode()
{
}

void EdieRepublishingOdomNode::OdometryCallback(const nav_msgs::msg::Odometry &odom)
{
    odom_sub_ = odom;

    // 측정된 처음 위치 RPY 초기화
    // tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
    //                 odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    // tf2::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);
    double yaw = odom_utils_.yawFromQuat(odom.pose.pose.orientation);
    
    // 초기 조건 설정
    if(HandleFirstOdom()) return;

    geometry_msgs::msg::Pose raw_pose_ = odom_utils_.GetPose(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, odom_utils_.NormalizeAngleToPi(yaw));
    odom_utils_.SetPubOdomInfo(odom, raw_odom_pub_, raw_pose_, "raw");
    // odom → raw TF 브로드캐스트   
    geometry_msgs::msg::TransformStamped t_raw;
    BroadcastTF(t_raw, raw_odom_pub_, "raw");

    // 휠 인코더 오도메트리 퍼블리셔
    PubRawOdom();
}

bool EdieRepublishingOdomNode::HandleFirstOdom()
{
    // 초기 조건 설정
    if(is_first_odom_)
    {        
        // 공분산 초기화
        std::fill(raw_odom_pub_.pose.covariance.begin(), raw_odom_pub_.pose.covariance.end(), 0.0);
        std::fill(raw_odom_pub_.twist.covariance.begin(), raw_odom_pub_.twist.covariance.end(), 0.0);
          
        // 측정 노이즈 공분산 설정 [고정값]
        // x, y, z, roll, pitch, yaw
        raw_odom_pub_.pose.covariance[0] = 5e-2; // x
        raw_odom_pub_.pose.covariance[7] = 5e-2; // y
        raw_odom_pub_.pose.covariance[14] = 1e-9; // z
        raw_odom_pub_.pose.covariance[21] = 1e-9; // roll
        raw_odom_pub_.pose.covariance[28] = 1e-9; // pitch
        raw_odom_pub_.pose.covariance[35] = 6e-2; // yaw
        // vx, vy, vz, vroll, vpitch, vyaw
        raw_odom_pub_.twist.covariance[0] = 5e-4; // vx
        raw_odom_pub_.twist.covariance[7] = 1e-9; // vy
        raw_odom_pub_.twist.covariance[14] = 1e-9; // vz
        raw_odom_pub_.twist.covariance[21] = 1e-9; // vroll
        raw_odom_pub_.twist.covariance[28] = 1e-9; // vpitch
        raw_odom_pub_.twist.covariance[35] = 7e-4; //6.5e-4; //5e-4; // vyaw

        is_first_odom_ = false;
        return true;
    }
    return false;
}

// 입력: 오도메트리 메시지
// 출력: TF 브로드캐스트
void EdieRepublishingOdomNode::BroadcastTF(
    geometry_msgs::msg::TransformStamped &t, 
    const nav_msgs::msg::Odometry &target_odom, 
    std::string child_frame_id)
{
    t.header.frame_id = target_odom.header.frame_id;            // "odom"
    t.header.stamp    = target_odom.header.stamp;               // 시간 동기
    t.child_frame_id  = child_frame_id;

    // modeled_odom_.pose 에 담긴 position/orientation 을 그대로 사용
    t.transform.translation.x = target_odom.pose.pose.position.x;
    t.transform.translation.y = target_odom.pose.pose.position.y;
    t.transform.translation.z = target_odom.pose.pose.position.z;
    t.transform.rotation      = target_odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
}

// 휠 인코더 오도메트리 퍼블리셔
void EdieRepublishingOdomNode::PubRawOdom()
{
    pub_raw_odom->publish(raw_odom_pub_);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EdieRepublishingOdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}