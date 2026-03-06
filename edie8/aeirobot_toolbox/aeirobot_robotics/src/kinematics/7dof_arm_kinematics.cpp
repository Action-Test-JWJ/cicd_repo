#include "aeirobot_robotics/kinematics/7dof_arm_kinematics.h"

/*********************************************************
     _      ____    __  __
    / \    |  _ \  |  \/  |
   / _ \   | |_) | | |\/| |
  / ___ \  |  _ <  | |  | |
 /_/   \_\ |_| \_\ |_|  |_|

*********************************************************/
namespace aeirobot
{
  ArmKinematics::ArmKinematics(double link_0, double link_1, double link_3, double link_5, double link_7, double epsilon, bool left_arm)
  {
    l0 = link_0;  // pelvis ~ chest
    l1 = link_1;  // chest ~ shoulder
    l3 = link_3;  // shoulder ~ elbow
    l5 = link_5;  // elbow ~ wrist 
    l7 = link_7;  // wrist ~ hand
    k_epsilon = epsilon;
    is_left_arm = left_arm;
  }
  ArmKinematics::~ArmKinematics() {}

  void ArmKinematics::SetLinkLength(double link_0, double link_1, double link_3, double link_5, double link_7)
  {
    l0 = link_0;
    l1 = link_1;
    l3 = link_3;
    l5 = link_5;
    l7 = link_7;
  }

  std::pair<bool, std::vector<double>> ArmKinematics::CalcInverseKinematics(bool auto_on, double x, double y, double z, double roll, double pitch, double yaw, double elbow_degree)
  {
    std::vector<double> joint_angles;
    joint_angles.resize(7);
    float theta[7] = {0, 0, 0, 0, 0, 0, 0};
    //-------------------------------------------------------------------------------------------------------------
    Eigen::Vector3d position_mat_root_to_end(x, y, z);
    Eigen::Vector3d euler_end(aeirobot::DegToRad(roll), aeirobot::DegToRad(pitch), aeirobot::DegToRad(yaw));
    Eigen::Vector3d position_root_to_end = Eigen::Vector3d(x, y - l1, z - l0);
    if (is_left_arm)
    {
      position_root_to_end = Eigen::Vector3d(x, y - l1, z - l0);
    }
    else
    {
      position_root_to_end = Eigen::Vector3d(x, y + l1, z - l0);
    }
    // std::cout << "------------------------------------------------------" << std::endl;
    // std::cout << position_root_to_end << std::endl;
    // Eigen::Vector3d euler_end(aeirobot::DegToRad(roll), aeirobot::DegToRad(pitch), aeirobot::DegToRad(yaw));
    // std::cout << "x: " << position_mat_root_to_end(0) << " y: " << position_mat_root_to_end(1) << " z: " << position_mat_root_to_end(2) << std::endl;
    // std::cout << "roll: " << aeirobot::RadToDeg(euler_end(0)) << " pitch: " << aeirobot::RadToDeg(euler_end(1)) << " yaw: " << aeirobot::RadToDeg(euler_end(2)) << std::endl;
    //-----------------------------------------------------------------------------------------------------------
    Eigen::Matrix3d rotation_mat_base_to_dh; // robot 기본 좌표계를 dh 파라미터 설정시의 좌표계로 변경해주는 회전 행렬
    Eigen::Matrix4d transform_mat_root_to_joint1;
    if (is_left_arm)
    {
      rotation_mat_base_to_dh << -1, 0, 0,
          0, -1, 0,
          0, 0, 1;
      transform_mat_root_to_joint1 << 1, 0, 0, 0,
          0, 0, 1, l1,
          0, -1, 0, l0,
          0, 0, 0, 1;
    }
    else
    {
      rotation_mat_base_to_dh << -1, 0, 0,
          0, 1, 0,
          0, 0, -1;
      transform_mat_root_to_joint1 << 1, 0, 0, 0,
          0, 0, 1, -l1,
          0, -1, 0, l0,
          0, 0, 0, 1;
    }

    Eigen::Matrix4d transform_mat_root_to_end = Eigen::Matrix4d::Identity();
    // transform_mat_root_to_end.block<3, 3>(0, 0) = EulerAnglesToRotationMatrix(euler_end) * rotation_mat_base_to_dh;
    Eigen::Matrix3d rotation_part_of_transformation = aeirobot::GetTransformationMatrix(0, 0, 0,
                                                                                       aeirobot::DegToRad(roll), aeirobot::DegToRad(pitch), aeirobot::DegToRad(yaw))
                                                          .block<3, 3>(0, 0);
    transform_mat_root_to_end.block<3, 3>(0, 0) = rotation_part_of_transformation * rotation_mat_base_to_dh;
    transform_mat_root_to_end.block<3, 1>(0, 3) = position_mat_root_to_end;
    Eigen::Matrix4d transform_mat_joint1_to_end = transform_mat_root_to_joint1.inverse() * transform_mat_root_to_end;
    // std::cout << "--R_e----------------------------------------" << std::endl;
    // std::cout << EulerAnglesToRotationMatrix(euler_end) << std::endl;
    // std::cout << "--transform_mat_root_to_end----------------------------------------" << std::endl;
    // std::cout << transform_mat_root_to_end << std::endl;
    // std::cout << "--transform_mat_joint1_to_end----------------------------------------" << std::endl;
    // std::cout << transform_mat_joint1_to_end << std::endl;
    //-----------------------------------------------------------------------------------------------------------
    Eigen::Matrix3d rotation_mat_joint1_to_end = transform_mat_joint1_to_end.block<3, 3>(0, 0);
    Eigen::Vector3d position_mat_joint1_to_end = transform_mat_joint1_to_end.block<3, 1>(0, 3);
    Eigen::Vector3d mat_link7(-l7, 0, 0);
    Eigen::Vector3d position_mat_joint1_to_wrist = position_mat_joint1_to_end - rotation_mat_joint1_to_end * mat_link7;
    if (abs(position_mat_joint1_to_wrist(k_x)) < k_epsilon)
      position_mat_joint1_to_wrist(k_x) = 0.0;
    if (abs(position_mat_joint1_to_wrist(k_y)) < k_epsilon)
      position_mat_joint1_to_wrist(k_y) = 0.0;
    if (abs(position_mat_joint1_to_wrist(k_z)) < k_epsilon)
      position_mat_joint1_to_wrist(k_z) = 0.0;
    // std::cout << "========================================" << std::endl;
    // std::cout << position_mat_joint1_to_wrist << std::endl;
    //-----------------------------------------------------------------------------------------------------------
    double c_e = (pow(l3, 2) + pow(l5, 2) - pow(position_mat_joint1_to_wrist.norm(), 2)) / (2 * l3 * l5);
    double s_e = sqrt(1 - pow(c_e, 2));
    if (abs(c_e) >= 1)
      s_e = 0.0;
    if (abs(c_e) < k_epsilon)
      c_e = 0;
    if (abs(s_e) < k_epsilon)
      s_e = 0;

    if (is_left_arm)
    {
      theta[k_joint_4] = (M_PI - atan2(s_e, c_e));
    }
    else
    {
      theta[k_joint_4] = (-M_PI + atan2(s_e, c_e));
    }

    // std::cout << "c_e: " << c_e << " s_e: " << s_e << std::endl;
    //-----------------------------------------------------------------------------------------------------------
    Eigen::Vector3d vector_gravity(-1, 0, 0);
    Eigen::Vector3d vector_ground(0, 0, 1);
    Eigen::Vector3d vector_normal = position_mat_joint1_to_wrist / position_mat_joint1_to_wrist.norm();
    Eigen::Vector3d vector_axis_u = (vector_gravity - (vector_gravity.dot(vector_normal)) * vector_normal) / (vector_gravity - (vector_gravity.dot(vector_normal)) * vector_normal).norm();
    if (position_mat_joint1_to_wrist.norm() == l3 + l5)
      vector_axis_u = Eigen::Vector3d::Zero();
    Eigen::Vector3d vector_axis_v = vector_normal.cross(vector_axis_u);
    double c_a = (pow(position_mat_joint1_to_wrist.norm(), 2) + pow(l3, 2) - pow(l5, 2)) / (2 * position_mat_joint1_to_wrist.norm() * l3);
    if (position_mat_joint1_to_wrist.norm() == l3 + l5)
      c_a = 1;
    double s_a = sqrt(1 - pow(c_a, 2));
    if (abs(c_a) >= 1)
      s_a = 0;

    float elbow_degree_modified = 0;
    if (auto_on)
    {
      float angle_u_and_ground = aeirobot::RadToDeg(atan2((vector_axis_u.cross(vector_ground).dot(vector_normal)),
                                                          (vector_axis_u.dot(vector_ground))));
      if(is_left_arm)
        elbow_degree_modified = angle_u_and_ground -90 + elbow_degree;
      else
        elbow_degree_modified = angle_u_and_ground -90 - elbow_degree;
    }
    else
      elbow_degree_modified = elbow_degree;

    double c_pi = cos(aeirobot::DegToRad(elbow_degree_modified));
    double s_pi = sin(aeirobot::DegToRad(elbow_degree_modified));
    
    Eigen::Vector3d position_mat_joint1_to_elbow = l3 * c_a * vector_normal + l3 * s_a * (c_pi * vector_axis_u + s_pi * vector_axis_v);
    if (abs(position_mat_joint1_to_elbow(k_x)) < k_epsilon)
      position_mat_joint1_to_elbow(k_x) = 0;
    if (abs(position_mat_joint1_to_elbow(k_y)) < k_epsilon)
      position_mat_joint1_to_elbow(k_y) = 0;
    if (abs(position_mat_joint1_to_elbow(k_z)) < k_epsilon)
      position_mat_joint1_to_elbow(k_z) = 0;
    // std::cout << "c_a: " << c_e << " s_a: " << s_e <<" position_mat_joint1_to_wrist.norm: " << position_mat_joint1_to_wrist.norm() << std::endl;
    // std::cout << "position_mat_joint1_to_elbow ========================================" << std::endl;
    // std::cout << position_mat_joint1_to_elbow << std::endl;
    //-----------------------------------------------------------------------------------------------------------

    if (is_left_arm)
    {
      theta[k_joint_1] = atan2(position_mat_joint1_to_elbow(k_y), position_mat_joint1_to_elbow(k_x));
      theta[k_joint_2] = atan2(-sqrt(pow(position_mat_joint1_to_elbow(k_x), 2) + pow(position_mat_joint1_to_elbow(k_y), 2)), position_mat_joint1_to_elbow(k_z));
    }
    else
    {
      theta[k_joint_1] = atan2(position_mat_joint1_to_elbow(k_y), position_mat_joint1_to_elbow(k_x));
      theta[k_joint_2] = atan2(sqrt(pow(position_mat_joint1_to_elbow(k_x), 2) + pow(position_mat_joint1_to_elbow(k_y), 2)), -position_mat_joint1_to_elbow(k_z));
    }
    //-----------------------------------------------------------------------------------------------------------
    double c1 = cos(theta[k_joint_1]);
    double c2 = cos(theta[k_joint_2]);
    double c4 = cos(theta[k_joint_4]);
    double s1 = sin(theta[k_joint_1]);
    double s2 = sin(theta[k_joint_2]);
    double s4 = sin(theta[k_joint_4]);
    if (abs(c1) < k_epsilon)
      c1 = 0;
    if (abs(c2) < k_epsilon)
      c2 = 0;
    if (abs(c4) < k_epsilon)
      c4 = 0;
    if (abs(s1) < k_epsilon)
      s1 = 0;
    if (abs(s2) < k_epsilon)
      s2 = 0;
    if (abs(s4) < k_epsilon)
      s4 = 0;
    Eigen::Vector3d position_mat_joint1_to_wrist_modified;
    if (is_left_arm)
    {
      position_mat_joint1_to_wrist_modified(k_x) = position_mat_joint1_to_wrist(0) + l3 * c1 * s2 + l5 * c1 * s2 * c4;
      position_mat_joint1_to_wrist_modified(k_y) = position_mat_joint1_to_wrist(1) + l3 * s1 * s2 + l5 * s1 * s2 * c4;
      position_mat_joint1_to_wrist_modified(k_z) = position_mat_joint1_to_wrist(2) - l3 * c2 - l5 * c2 * c4;
    }
    else
    {
      position_mat_joint1_to_wrist_modified(k_x) = position_mat_joint1_to_wrist(0) - l3 * c1 * s2 - l5 * c1 * s2 * c4;
      position_mat_joint1_to_wrist_modified(k_y) = position_mat_joint1_to_wrist(1) - l3 * s1 * s2 - l5 * s1 * s2 * c4;
      position_mat_joint1_to_wrist_modified(k_z) = position_mat_joint1_to_wrist(2) + l3 * c2 + l5 * c2 * c4;
    }

    if (abs(position_mat_joint1_to_wrist_modified(k_x)) < k_epsilon)
      position_mat_joint1_to_wrist_modified(k_x) = 0;
    if (abs(position_mat_joint1_to_wrist_modified(k_y)) < k_epsilon)
      position_mat_joint1_to_wrist_modified(k_y) = 0;
    if (abs(position_mat_joint1_to_wrist_modified(k_z)) < k_epsilon)
      position_mat_joint1_to_wrist_modified(k_z) = 0;
    theta[k_joint_3] = atan2((position_mat_joint1_to_wrist_modified(k_x) * s1 - position_mat_joint1_to_wrist_modified(k_y) * c1), (position_mat_joint1_to_wrist_modified(k_x) * c1 * c2 + position_mat_joint1_to_wrist_modified(k_y) * s1 * c2 + position_mat_joint1_to_wrist_modified(k_z) * s2));
    // std::cout << position_mat_joint1_to_wrist << std::endl;
    // std::cout << "position_mat_joint1_to_wristx-: "<<position_mat_joint1_to_wrist_modified(k_x)<<" position_mat_joint1_to_wrist_modified(k_y): "<<position_mat_joint1_to_wrist_modified(k_y)<<" position_mat_joint1_to_wrist_modified(k_z): "<<position_mat_joint1_to_wrist_modified(k_z)<<std::endl;
    // std::cout << "theta[k_joint_3]: " << theta[k_joint_3] <<" atan2 1: " << (position_mat_joint1_to_wrist_modified(k_x)*s1 - position_mat_joint1_to_wrist_modified(k_y)*c1) << " atan2 2: " << (position_mat_joint1_to_wrist_modified(k_x)*c1*c2 + position_mat_joint1_to_wrist_modified(k_y)*s1*c2 + position_mat_joint1_to_wrist_modified(k_z)*s2) << std::endl;
    // std::cout<<"c1: "<<c1<<" c2: "<<c2<<" c4: "<<c4<<std::endl;
    // std::cout<<"s1: "<<s1<<" s2: "<<s2<<" s4: "<<s4<<std::endl;
    //-----------------------------------------------------------------------------------------------------------
    //팔을 쭉 뻗었을때 예외처리
    Eigen::Vector3d position_mat_shoulder(0, 0, 0);
    Eigen::Vector3d position_mat_elbow = position_mat_joint1_to_elbow;
    Eigen::Vector3d position_mat_wrist = position_mat_joint1_to_wrist;
    Eigen::Vector3d v1 = (position_mat_elbow - position_mat_shoulder).normalized();
    Eigen::Vector3d v2 = (position_mat_wrist - position_mat_elbow).normalized();
    Eigen::Vector3d elbowPlaneNormal = v1.cross(v2);
    if (elbowPlaneNormal.norm() < k_epsilon) 
        theta[k_joint_3] = prev_theta[k_joint_3]; 
    else 
        prev_theta[k_joint_3] = theta[k_joint_3];
    //-----------------------------------------------------------------------------------------------------------
    double c3 = cos(theta[k_joint_3]);
    double s3 = sin(theta[k_joint_3]);
    if (abs(c3) < k_epsilon)
      c3 = 0;
    if (abs(s3) < k_epsilon)
      s3 = 0;

    Eigen::Matrix4d transform_mat_joint1_to_joint5 = Eigen::Matrix4d::Zero();
    if (is_left_arm)
    {
      transform_mat_joint1_to_joint5(0, 0) = c4 * (s1 * s3 + c1 * c2 * c3) + c1 * s2 * s4;
      transform_mat_joint1_to_joint5(0, 1) = c3 * s1 - c1 * c2 * s3;
      transform_mat_joint1_to_joint5(0, 2) = c1 * c4 * s2 - s4 * (s1 * s3 + c1 * c2 * c3);
      transform_mat_joint1_to_joint5(0, 3) = -c1 * l3 * s2;

      transform_mat_joint1_to_joint5(1, 0) = s1 * s2 * s4 - c4 * (c1 * s3 - c2 * c3 * s1);
      transform_mat_joint1_to_joint5(1, 1) = -c1 * c3 - c2 * s1 * s3;
      transform_mat_joint1_to_joint5(1, 2) = s4 * (c1 * s3 - c2 * c3 * s1) + c4 * s1 * s2;
      transform_mat_joint1_to_joint5(1, 3) = -l3 * s1 * s2;

      transform_mat_joint1_to_joint5(2, 0) = c3 * c4 * s2 - c2 * s4;
      transform_mat_joint1_to_joint5(2, 1) = -s2 * s3;
      transform_mat_joint1_to_joint5(2, 2) = -c2 * c4 - c3 * s2 * s4;
      transform_mat_joint1_to_joint5(2, 3) = c2 * l3;
    }
    else
    {
      transform_mat_joint1_to_joint5(0, 0) = c4 * (s1 * s3 + c1 * c2 * c3) + c1 * s2 * s4;
      transform_mat_joint1_to_joint5(0, 1) = c3 * s1 - c1 * c2 * s3;
      transform_mat_joint1_to_joint5(0, 2) = c1 * c4 * s2 - s4 * (s1 * s3 + c1 * c2 * c3);
      transform_mat_joint1_to_joint5(0, 3) = c1 * l3 * s2;

      transform_mat_joint1_to_joint5(1, 0) = s1 * s2 * s4 - c4 * (c1 * s3 - c2 * c3 * s1);
      transform_mat_joint1_to_joint5(1, 1) = -c1 * c3 - c2 * s1 * s3;
      transform_mat_joint1_to_joint5(1, 2) = s4 * (c1 * s3 - c2 * c3 * s1) + c4 * s1 * s2;
      transform_mat_joint1_to_joint5(1, 3) = l3 * s1 * s2;

      transform_mat_joint1_to_joint5(2, 0) = c3 * c4 * s2 - c2 * s4;
      transform_mat_joint1_to_joint5(2, 1) = -s2 * s3;
      transform_mat_joint1_to_joint5(2, 2) = -c2 * c4 - c3 * s2 * s4;
      transform_mat_joint1_to_joint5(2, 3) = -c2 * l3;
    }
    transform_mat_joint1_to_joint5(3, 0) = 0;
    transform_mat_joint1_to_joint5(3, 1) = 0;
    transform_mat_joint1_to_joint5(3, 2) = 0;
    transform_mat_joint1_to_joint5(3, 3) = 1;
    Eigen::Matrix4d transform_mat_joint5_to_end = transform_mat_joint1_to_joint5.inverse() * transform_mat_joint1_to_end;

    double r13 = transform_mat_joint5_to_end(0, 2);
    double r23 = transform_mat_joint5_to_end(1, 2);
    double r31 = transform_mat_joint5_to_end(2, 0);
    double r32 = transform_mat_joint5_to_end(2, 1);
    double r33 = transform_mat_joint5_to_end(2, 2);
    if (abs(r13) < k_epsilon)
      r13 = 0;
    if (abs(r23) < k_epsilon)
      r23 = 0;
    if (abs(r31) < k_epsilon)
      r31 = 0;
    if (abs(r32) < k_epsilon)
      r32 = 0;
    if (abs(r33) < k_epsilon)
      r33 = 0;

    if (is_left_arm)
    {
      theta[k_joint_5] = atan2(r23, r13);
      theta[k_joint_6] = atan2(sqrt(pow(r31, 2) + pow(r32, 2)), -r33);
      theta[k_joint_7] = atan2(-r32, r31);
    }
    else
    {
      theta[k_joint_5] = atan2(-r23, -r13);
      theta[k_joint_6] = atan2(-sqrt(pow(r31, 2) + pow(r32, 2)), -r33);
      theta[k_joint_7] = atan2(r32, -r31);
    }
    //-----------------------------------------------------------------------------------------------------------
    bool ik_result = false;
    if (std::isnan(theta[k_joint_1]) || std::isnan(theta[k_joint_2]) || std::isnan(theta[k_joint_3]) || std::isnan(theta[k_joint_4]) || std::isnan(theta[k_joint_5]) || std::isnan(theta[k_joint_6]) || std::isnan(theta[k_joint_7]))
    {
      ik_result = false;
    }
    else
    {
      ik_result = true;
      if (abs(theta[k_joint_1]) < k_epsilon)
        theta[k_joint_1] = 0;
      if (abs(theta[k_joint_2]) < k_epsilon)
        theta[k_joint_2] = 0;
      if (abs(theta[k_joint_3]) < k_epsilon)
        theta[k_joint_3] = 0;
      if (abs(theta[k_joint_4]) < k_epsilon)
        theta[k_joint_4] = 0;
      if (abs(theta[k_joint_5]) < k_epsilon)
        theta[k_joint_5] = 0;
      if (abs(theta[k_joint_6]) < k_epsilon)
        theta[k_joint_6] = 0;
      if (abs(theta[k_joint_7]) < k_epsilon)
        theta[k_joint_7] = 0;
    }
    // std::cout << "/////////////////////////////////////////////////////////////////////////" << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_1]: " << aeirobot::RadToDeg(theta[k_joint_1]) << " theta[k_joint_2]: " << aeirobot::RadToDeg(theta[k_joint_2]) << " theta[k_joint_3]: " << aeirobot::RadToDeg(theta[k_joint_3]) << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_4]: " << aeirobot::RadToDeg(theta[k_joint_4]) << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_5]: " << aeirobot::RadToDeg(theta[k_joint_5]) << " theta[k_joint_6]: " << aeirobot::RadToDeg(theta[k_joint_6]) << " theta[k_joint_7]: " << aeirobot::RadToDeg(theta[k_joint_7]) << std::endl;
    if (is_left_arm)
    {
      joint_angles[0] = ((theta[k_joint_1] - M_PI / 2));
      joint_angles[1] = ((theta[k_joint_2] + M_PI / 2));
      joint_angles[2] = ((theta[k_joint_3] - M_PI / 2));
      joint_angles[3] = -((theta[k_joint_4]));
      joint_angles[4] = ((theta[k_joint_5]));
      joint_angles[5] = -((theta[k_joint_6] - M_PI / 2));
      joint_angles[6] = ((theta[k_joint_7]));
    }
    else
    {
      joint_angles[0] = -((theta[k_joint_1] - M_PI / 2));
      joint_angles[1] = ((theta[k_joint_2] - M_PI / 2));
      joint_angles[2] = -((theta[k_joint_3] - M_PI / 2));
      joint_angles[3] = -((theta[k_joint_4]));
      joint_angles[4] = -((theta[k_joint_5]));
      joint_angles[5] = -((theta[k_joint_6] + M_PI / 2));
      joint_angles[6] = -((theta[k_joint_7]));
    }
    for (int i = 0; i < 7; ++i)
    {
      joint_angles[i] = aeirobot::NormalizeRadian(joint_angles[i]);
    }
    // std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_1]: " << joint_angles[k_joint_1] << " theta[k_joint_2]: " << joint_angles[k_joint_2] << " theta[k_joint_3]: " << joint_angles[k_joint_3] << std::endl;
    // std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_4]: " << joint_angles[k_joint_4] << std::endl;
    //  std::cout << std::fixed << std::setprecision(2) << "theta[k_joint_5]: " << joint_angles[k_joint_5] << " theta[k_joint_6]: " << joint_angles[k_joint_6] << " theta[k_joint_7]: " << joint_angles[k_joint_7] << std::endl;
    return {ik_result, joint_angles};
  }

  aeirobot_msgs::msg::PoseXYZRPY ArmKinematics::CalcForwardKinematics(const std::vector<double> &joint_angles)
  {
    if (joint_angles.size() != 7)
    {
      throw std::invalid_argument("Joint angles vector must have 7 elements.");
    }

    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1];
    double theta3 = joint_angles[2];
    double theta4 = joint_angles[3];
    double theta5 = joint_angles[4];
    double theta6 = joint_angles[5];
    double theta7 = joint_angles[6];

    Eigen::Matrix4d transform_mat_world_to_base;
    Eigen::Matrix4d transform_mat_base_to_joint1;
    Eigen::Matrix4d transform_mat_joint1_to_joint2;
    Eigen::Matrix4d transform_mat_joint2_to_joint3;
    Eigen::Matrix4d transform_mat_joint3_to_joint4;
    Eigen::Matrix4d transform_mat_joint4_to_joint5;
    Eigen::Matrix4d transform_mat_joint5_to_joint6;
    Eigen::Matrix4d transform_mat_joint6_to_joint7;
    Eigen::Matrix4d transform_mat_joint7_to_end;

    if (is_left_arm)
    {
      // 회전축 변환
      theta1 += M_PI / 2;
      theta2 -= M_PI / 2;
      theta3 += M_PI / 2;
      theta4 = -theta4;
      theta6 = -theta6 + M_PI / 2;

      // 왼손 기준 변환 행렬
      transform_mat_world_to_base = aeirobot::DHMatrix(-M_PI / 2, 0, l0, 0);
      transform_mat_base_to_joint1 = aeirobot::DHMatrix(0, 0, l1, 0);
      transform_mat_joint1_to_joint2 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta1);
      transform_mat_joint2_to_joint3 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta2);
      transform_mat_joint3_to_joint4 = aeirobot::DHMatrix(M_PI / 2, 0.0, -l3, theta3);
      transform_mat_joint4_to_joint5 = aeirobot::DHMatrix(-M_PI / 2, 0.0, 0.0, theta4);
      transform_mat_joint5_to_joint6 = aeirobot::DHMatrix(M_PI / 2, 0.0, -l5, theta5);
      transform_mat_joint6_to_joint7 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta6);
      transform_mat_joint7_to_end = aeirobot::DHMatrix(0.0, -l7, 0.0, theta7);
    }
    else
    {
      // 회전축 변환
      theta1 = -theta1 + M_PI / 2;
      theta2 += M_PI / 2;
      theta3 = -theta3 + M_PI / 2;
      theta4 = -theta4;
      theta5 = -theta5;
      theta6 = -theta6 - M_PI / 2;
      theta7 = -theta7;

      // 오른손 기준 변환 행렬
      transform_mat_world_to_base = aeirobot::DHMatrix(-M_PI / 2, 0, l0, 0);
      transform_mat_base_to_joint1 = aeirobot::DHMatrix(0, 0, -l1, 0);
      transform_mat_joint1_to_joint2 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta1);
      transform_mat_joint2_to_joint3 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta2);
      transform_mat_joint3_to_joint4 = aeirobot::DHMatrix(M_PI / 2, 0.0, l3, theta3);
      transform_mat_joint4_to_joint5 = aeirobot::DHMatrix(-M_PI / 2, 0.0, 0.0, theta4);
      transform_mat_joint5_to_joint6 = aeirobot::DHMatrix(M_PI / 2, 0.0, l5, theta5);
      transform_mat_joint6_to_joint7 = aeirobot::DHMatrix(M_PI / 2, 0.0, 0.0, theta6);
      transform_mat_joint7_to_end = aeirobot::DHMatrix(0.0, -l7, 0.0, theta7);
    }

    // 전체 변환 행렬 계산
    Eigen::Matrix4d transform_mat_world_to_end = transform_mat_world_to_base * transform_mat_base_to_joint1 *
                                                 transform_mat_joint1_to_joint2 * transform_mat_joint2_to_joint3 *
                                                 transform_mat_joint3_to_joint4 * transform_mat_joint4_to_joint5 *
                                                 transform_mat_joint5_to_joint6 * transform_mat_joint6_to_joint7 *
                                                 transform_mat_joint7_to_end;

    // 엔드 이펙터의 위치와 방향을 계산
    Eigen::Vector3d position = transform_mat_world_to_end.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = transform_mat_world_to_end.block<3, 3>(0, 0);

    if (is_left_arm)
    {
      Eigen::Matrix3d rotation_z_180;
      rotation_z_180 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      rotation = rotation * rotation_z_180;
    }
    else
    {
      Eigen::Matrix3d rotation_y_180;
      rotation_y_180 = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
      rotation = rotation * rotation_y_180;
    }

    // 매우 작은 값은 0으로 변환
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        if (std::abs(rotation(i, j)) < 1e-3)
        {
          rotation(i, j) = 0.0;
        }
      }
    }

    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0); // ZYX 순서로 오일러 각 추출

    aeirobot_msgs::msg::PoseXYZRPY pose;
    pose.x = position(0);
    pose.y = position(1);
    pose.z = position(2);
    pose.roll = euler_angles(2);  // 롤
    pose.pitch = euler_angles(1); // 피치
    pose.yaw = euler_angles(0);   // 요

    return pose;
  }

  double ArmKinematics::CalcElbowDegree(geometry_msgs::msg::Pose endpoint, geometry_msgs::msg::Pose elbow){
    double elbow_degree = 0;
    //---------------------------------------------------------------------------------
    // Eigen::Matrix4d transform_mat_root_to_joint1;
    // if (is_left_arm)
    // {
      // transform_mat_root_to_joint1 << 1, 0, 0, 0,
          // 0, 0, 1, l1,
          // 0, -1, 0, l0,
          // 0, 0, 0, 1;
    // }
    // else
    // {
      // transform_mat_root_to_joint1 << 1, 0, 0, 0,
          // 0, 0, 1, -l1,
          // 0, -1, 0, l0,
          // 0, 0, 0, 1;
    // }
    // Eigen::Vector3d position_mat_root_to_end(x, y, z);
    // Eigen::Matrix4d transform_mat_root_to_end = aeirobot::PoseToTransformMatrix(endpoint);
    // Eigen::Matrix4d transform_mat_joint1_to_end = transform_mat_root_to_joint1.inverse() * transform_mat_root_to_end;
    //---------------------------------------------------------------------------------
    // Eigen::Matrix3d rotation_mat_joint1_to_end = transform_mat_joint1_to_end.block<3, 3>(0, 0);
    // Eigen::Vector3d position_mat_joint1_to_end = transform_mat_joint1_to_end.block<3, 1>(0, 3);
    // Eigen::Vector3d mat_link7(-l7, 0, 0);
    // Eigen::Vector3d position_mat_joint1_to_wrist = position_mat_joint1_to_end - rotation_mat_joint1_to_end * mat_link7;
    //---------------------------------------------------------------------------------
    // Eigen::Vector3d vector_gravity(-1, 0, 0);
    // Eigen::Vector3d vector_normal = position_mat_joint1_to_wrist / position_mat_joint1_to_wrist.norm();
    // Eigen::Vector3d vector_axis_u = (vector_gravity - (vector_gravity.dot(vector_normal)) * vector_normal) / (vector_gravity - (vector_gravity.dot(vector_normal)) * vector_normal).norm();
    //---------------------------------------------------------------------------------
    return elbow_degree;
  }
}