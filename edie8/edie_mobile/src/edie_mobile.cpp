#include "edie_mobile/edie_mobile_main.hpp"

EdieMobileNode::EdieMobileNode() : Node("edie_mobile_node")
{
  operation_mode = 1; // 0이 auto mode, 1,2,3이 manual mode 
  speed.angular.z = 0.0;
  speed.linear.x = 0.0;
  goal_point = 0;

  e_s = 0;
  e_theta = 0;
  tracking_state = false;
  point_move_flag = false;
  goal_input_flag = false;
  direct_vel_active_ = false;

  DeclareParams();
  GetParams();
  
  ROS_BLUE_STREAM("Point1.x : " << Point1.x << " | Point1.y : " << Point1.y << " | Point1.theta : " << Point1.theta);
  ROS_BLUE_STREAM("Point2.x : " << Point2.x << " | Point2.y : " << Point2.y << " | Point2.theta : " << Point2.theta);
  ROS_BLUE_STREAM("Point3.x : " << Point3.x << " | Point3.y : " << Point3.y << " | Point3.theta : " << Point3.theta);
  ROS_BLUE_STREAM("Point4.x : " << Point4.x << " | Point4.y : " << Point4.y << " | Point4.theta : " << Point4.theta);

  rclcpp::QoS sub_qos(rclcpp::KeepLast(10));
  sub_qos.best_effort();
  sub_robot_pos = this->create_subscription<geometry_msgs::msg::PoseStamped>("/edie8/localization/robot_pose", sub_qos, std::bind(&EdieMobileNode::RobotPoseCallback, this, _1));
  sub_goal_pose = this->create_subscription<geometry_msgs::msg::PoseStamped>("/edie8/navigation/goal_pose", 10, std::bind(&EdieMobileNode::GoalPoseCallback, this, _1));
  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/edie8/diff_drive_controller/odom", 10, std::bind(&EdieMobileNode::OdomCallback, this, _1));
  sub_goal_point = this->create_subscription<std_msgs::msg::Int32>("/edie8/navigation/waypoint_number", 10, std::bind(&EdieMobileNode::WayPointNumberCallback, this, _1));
  sub_operation_mode = this->create_subscription<std_msgs::msg::UInt8>("/edie8/operation_mode", 10, std::bind(&EdieMobileNode::OperationModeCallback, this, _1));
  sub_direct_vel = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/edie8/navigation/direct_vel", 10, std::bind(&EdieMobileNode::DirectVelCallback, this, _1));

  // pub_odom_to_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/edie8/localization/robot_pose", 10);
  pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>("/edie8/diff_drive_controller/cmd_vel_unstamped", 10);
  pub_path = this->create_publisher<nav_msgs::msg::Path>("/edie8/navigation/path", 10);
}

void EdieMobileNode::DebugPrint()
{
  // printf("\033[2J");
  // printf("\033[1;1H");
  // std::cout << " cur_pos_x  : " << cur_pose_2D.x << "|"
            // << " cur_pos_y  : " << cur_pose_2D.y << "|"
            // << " cur_pos_theta  : " << RadianToDegree(cur_pose_2D.theta) << "|" << std::endl;

  // std::cout << " goal_pos_x : " << goal_pose_2D.x << "|"
            // << " goal_pos_y : " << goal_pose_2D.y << "|"
            // << " goal_pos_theta : " << RadianToDegree(goal_pose_2D.theta) << "|" << std::endl;

  if (!point_queue.empty())
  {
    std::cout << " front queue: " << point_queue.front() << "|" << std::endl;
  }
  // std::cout << " point_move_flag: " << point_move_flag << "|" << std::endl;

  // std::cout << " speed.linear.x: " << speed.linear.x << "|" << std::endl;
  // std::cout << " speed.angular.z: " << speed.angular.z << "|" << std::endl;
  // std::cout << "----------------------------------------------------------" << std::endl;
  // ROS_INFO("cur_pos_x : %.3f | cur_pos_y : %.3f | goal_pos_x : %.3f | goal_pos_y : %.3lf",
  //          cur_pose_2D.x, cur_pose_2D.y, goal_pose_2D.x,goal_pose_2D.y);
  // ROS_INFO("speed.linear.x : %.3lf | speed.angular.z: %.3lf", speed.linear.x, speed.angular.z);
}

void EdieMobileNode::RunProcess()
{
  DebugPrint();

  // 1. 고우선순위 직접 제어 명령 확인 (0.5초 타임아웃)
  if (direct_vel_active_ && !goal_input_flag)
  {
    pub_cmd_vel->publish(last_direct_vel_.twist);
    return; // 직접 제어 중에는 다른 로직을 실행하지 않음
  }
  direct_vel_active_ = false;
  // 2. 저우선순위 경로 추종 로직
  if (goal_input_flag)
  {
    PlanningPoint();
    MakePath(cur_pose_2D, goal_pose_2D);
    PubBezierCurve(line_path);
    DriveSeqence();
    pub_cmd_vel->publish(speed);
  }
  else
  {
    // 3. 아무런 명령이 없으면 정지
    SpeedZero();
    pub_cmd_vel->publish(speed);
  }
}

void EdieMobileNode::DriveSeqence()
{
  CalcErrors(cur_pose_2D.x, cur_pose_2D.y, cur_pose_2D.theta, goal_pose_2D.x, goal_pose_2D.y);
  if (abs(e_s) > dist_stop_condition) // 거리 밖에 있음
  {
    if ((abs(e_theta)) > 0.3)
    {
      speed.angular.z = theta_PID.process(e_theta);
      speed.linear.x = 0.0;
    }
    else
    {
      double steering;
      if (is_first_time == true)
      {
        steering = StanleyControl(cur_pose_2D.x, cur_pose_2D.y, cur_pose_2D.theta, max_vel, line_path);
        speed.linear.x = CalcLinearVel(steering, abs(e_s), abs(e_theta));
        speed.angular.z = steering;
        is_first_time = false;
      }
      else
      {
        steering = StanleyControl(cur_pose_2D.x, cur_pose_2D.y, cur_pose_2D.theta, speed.linear.x, line_path);
        speed.linear.x = CalcLinearVel(steering, abs(e_s), abs(e_theta));
        speed.angular.z = steering;
      }
    }
  }
  else if (abs(e_s) < dist_stop_condition) //거리 안에 있음
  {
    dis_done = true;
    CalcErrorsTheta(cur_pose_2D.theta, goal_pose_2D.theta);
    if ((abs(e_theta)) > rotational_stop_condition)
    {
      // if (yaw_check)
      // {
        // RCLCPP_INFO(this->get_logger(), "cur_pose_2D.theta: %f | goal_pose_2D.theta: %f | e_theta: %f", cur_pose_2D.theta, goal_pose_2D.theta, e_theta);
        speed.angular.z = theta_PID.process(e_theta) + e_theta * 1.0;
        speed.linear.x = 0.0;
      // }
    }
    else if ((abs(e_theta)) <= rotational_stop_condition)
    {
      SpeedZero();
      goal_input_flag = false;
    }
  }
}

void EdieMobileNode::CalcErrors(double cur_pos_x, double cur_pos_y, double cur_pos_theta, double goal_x, double goal_y)
{
  double delta_y_ref = goal_y - cur_pos_y;
  double delta_x_ref = goal_x - cur_pos_x;
  double delta_theta_ref;

  delta_theta_ref = atan2(delta_y_ref, delta_x_ref);
  e_s =
      sqrt(pow((delta_x_ref), 2) +
           pow((delta_y_ref),
               2));

  e_theta = delta_theta_ref - cur_pos_theta;

  if (abs(e_theta) > M_PI)
  {
    if (e_theta > 0.0)
      e_theta = e_theta - 2*M_PI;
    else
      e_theta = e_theta + 2*M_PI;
  }

  return;
}

void EdieMobileNode::CalcErrorsTheta(double cur_pos_theta, double goal_theta)
{
  e_theta = goal_theta - cur_pos_theta;

  if (abs(e_theta) > M_PI)
  {
    if (e_theta > 0.0)
      e_theta = e_theta - 2*M_PI;
    else
      e_theta = e_theta + 2*M_PI;
  }

  return;
}

void EdieMobileNode::SpeedZero()
{
  speed.angular.z = 0.0;
  speed.linear.x = 0.0;
}

void EdieMobileNode::DeclareParams()
{
  this->declare_parameter<double>("Point1_x", 0.0);
  this->declare_parameter<double>("Point1_y", 0.0);
  this->declare_parameter<double>("Point1_theta", 0.0);

  this->declare_parameter<double>("Point2_x", 0.0);
  this->declare_parameter<double>("Point2_y", 0.0);
  this->declare_parameter<double>("Point2_theta", 0.0);

  this->declare_parameter<double>("Point3_x", 0.0);
  this->declare_parameter<double>("Point3_y", 0.0);
  this->declare_parameter<double>("Point3_theta", 0.0);

  this->declare_parameter<double>("Point4_x", 0.0);
  this->declare_parameter<double>("Point4_y", 0.0);
  this->declare_parameter<double>("Point4_theta", 0.0);

  // stanley_linear_gain 파라미터 추가
  this->declare_parameter<double>("stanley_linear_gain", 0.0);

  this->declare_parameter<double>("rotational_PID.P", 0.0);
  this->declare_parameter<double>("rotational_PID.I", 0.0);
  this->declare_parameter<double>("rotational_PID.D", 0.0);
  this->declare_parameter<double>("rotational_PID.max_state", 0.0);
  this->declare_parameter<double>("rotational_PID.min_state", 0.0);

  this->declare_parameter<double>("translation_PID.P", 0.0);
  this->declare_parameter<double>("translation_PID.I", 0.0);
  this->declare_parameter<double>("translation_PID.D", 0.0);
  this->declare_parameter<double>("translation_PID.max_state", 0.0);
  this->declare_parameter<double>("translation_PID.min_state", 0.0);

  this->declare_parameter<double>("robot_stop.dist_stop_condition", 0.0);
  this->declare_parameter<double>("robot_stop.rotational_stop_condition", 0.0);
}

void EdieMobileNode::GetParams()
{
  this->get_parameter("Point1_x", Point1.x);
  this->get_parameter("Point1_y", Point1.y);
  this->get_parameter("Point1_theta", Point1.theta);

  this->get_parameter("Point2_x", Point2.x);
  this->get_parameter("Point2_y", Point2.y);
  this->get_parameter("Point2_theta", Point2.theta);

  this->get_parameter("Point3_x", Point3.x);
  this->get_parameter("Point3_y", Point3.y);
  this->get_parameter("Point3_theta", Point3.theta);

  this->get_parameter("Point4_x", Point4.x);
  this->get_parameter("Point4_y", Point4.y);
  this->get_parameter("Point4_theta", Point4.theta);

  // stanley_linear_gain 파라미터 추가
  this->get_parameter("stanley_linear_gain", stanley_linear_gain);

  this->get_parameter("rotational_PID.P", theta_PID.P);
  this->get_parameter("rotational_PID.I", theta_PID.I);
  this->get_parameter("rotational_PID.D", theta_PID.D);
  this->get_parameter("rotational_PID.max_state", theta_PID.max_state);
  this->get_parameter("rotational_PID.min_state", theta_PID.min_state);

  this->get_parameter("translation_PID.P", translation_PID.P);
  this->get_parameter("translation_PID.I", translation_PID.I);
  this->get_parameter("translation_PID.D", translation_PID.D);
  this->get_parameter("translation_PID.max_state", translation_PID.max_state);
  this->get_parameter("translation_PID.min_state", translation_PID.min_state);

  this->get_parameter("robot_stop.dist_stop_condition", dist_stop_condition);
  this->get_parameter("robot_stop.rotational_stop_condition", rotational_stop_condition);
}

EdieMobileNode::~EdieMobileNode()
{
}
