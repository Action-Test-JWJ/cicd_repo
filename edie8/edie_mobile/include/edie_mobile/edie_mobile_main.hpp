#ifndef mobile_control_
#define mobile_control_

#include "rclcpp/rclcpp.hpp"   
#include <yaml-cpp/yaml.h>

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "sensor_msgs/msg/joy.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

#include "aeirobot_toolbox/basic_tools.hpp"

#include <random>
#include <cmath>
#include <math.h>
#include <string>
#include <iostream>
#include <vector>
#include <queue>

#define bezier_resolution 50

#define k_gain 0.35
#define k_s 2.0
#define Max_Steering 1.3
// #define Length 0.385
#define Length 0.136

#define max_vel 0.25

#define constrain(amt, low, high) ((amt) <= (low) ? (low) : ((amt) >= (high) ? (high) : (amt)))

#define pi 3.141592
#define STOP_INTERACTION 100.0
#define EPSILON 0.0001
#define QUEUE_SIZE 3

using namespace std::chrono_literals;
using std::placeholders::_1;

double DegreeToRadian(double degree);
double RadianToDegree(double radian);
struct Pose2D
{
    double x, y, theta;
};
struct Path2D
{
    std::vector<double> x, y, theta;
};
struct ObjectPose
{
    Pose2D global, local;
};
double NormalizeAngle(double angle);

class PID
{
public:
    double P;
    double I;
    double D;
    double state_P;
    double state_I;
    double state_D;
    double state;
    double max_state;
    double min_state;
    double pre_state;
    double dt;
    double integrated_state;
    double pre_time;
    PID();
    ~PID();
    double process(double state);
};

class EdieMobileNode : public rclcpp::Node
{
public:
    bool block_state, tracking_state, point_move_flag, remote_state, yaw_check;

    bool goal_input_flag;

    double dist_stop_condition, rotational_stop_condition;

    int goal_point, near_point, min_index;

    double e_s;
    double e_theta;

    std::queue<int> point_queue;

    bool path_tendancy;
    bool is_first_time;
    bool dis_done;
    bool drive_state;

    bool lidar_enable;

    double steeer;
    double psi1;
    double cte_term1;
    double ref_yaw1;
    double cte11;
    double v1;

    EdieMobileNode();
    ~EdieMobileNode();

    void RunProcess();
    void DriveSeqence();
    void SpeedZero();
    void PlanningPoint();
    void DebugPrint();

    void CalcErrors(double cur_pos_x, double cur_pos_y, double cur_pos_theta, double goal_x, double goal_y);
    void CalcErrorsTheta(double cur_pos_theta, double goal_theta);

    void MakePath(Pose2D start, Pose2D end);
    void PubBezierCurve(Path2D bezier_curve);

    double CalculateDistance(const Pose2D &pointA, const Pose2D &pointB);

    double StanleyControl(double x, double y, double yaw, double v, Path2D path2d);
    double CalcLinearVel(double steer, double e_s, double e_theta);

    void DeclareParams();
    void GetParams();

    std::pair<double, double> CalculatePoint(double x, double y, double theta, double distance, double offset);

    PID theta_PID;
    PID translation_PID;

private:
    uint8_t operation_mode;

    Path2D line_path;

    Pose2D cur_pose_2D;
    Pose2D goal_pose_2D;

    double stanley_linear_gain;

    Pose2D Point1;
    Pose2D Point2;
    Pose2D Point3;
    Pose2D Point4;

    geometry_msgs::msg::Twist speed;

    // Publisher
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_odom_to_pose;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_robot_pos;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_goal_point;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_operation_mode;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_direct_vel;

    // void JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void RobotPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void GoalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void WayPointNumberCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void OperationModeCallback(const std_msgs::msg::UInt8::SharedPtr msg);
    void DirectVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    geometry_msgs::msg::TwistStamped last_direct_vel_;
    bool direct_vel_active_ = false;
};

#endif
