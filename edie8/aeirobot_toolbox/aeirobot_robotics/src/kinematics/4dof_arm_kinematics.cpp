#include "aeirobot_robotics/kinematics/4dof_arm_kinematics.h"

/*********************************************************
     _      ____    __  __
    / \    |  _ \  |  \/  |
   / _ \   | |_) | | |\/| |
  / ___ \  |  _ <  | |  | |
 /_/   \_\ |_| \_\ |_|  |_|

*********************************************************/
namespace aeirobot
{
  FourDofArmKinematics::FourDofArmKinematics(double link_0, double link_1, double link_2, double link_3, double epsilon, bool left_arm)
  {
    l0 = link_0;
    l1 = link_1;
    l2 = link_2;
    l3 = link_3;
    k_epsilon = epsilon;
    is_left_arm = left_arm;
  }
  
  FourDofArmKinematics::~FourDofArmKinematics() {}

  std::pair<bool, std::vector<double>> FourDofArmKinematics::CalcInverseKinematics(double x, double y, double z, double roll, double pitch, double yaw)
  {
    //-------------------------------------------------------------------------------------------------------------
    // Suppress unused parameter warnings
    (void)roll;
    (void)pitch;
    (void)yaw;
    
    std::vector<double> joint_angles;
    joint_angles.resize(4);
    float theta[4] = {0, 0, 0, 0};
    //-------------------------------------------------------------------------------------------------------------
    
    // Position vector from root to end effector
    Eigen::Vector3d position_root_to_end;
    if (is_left_arm)
    {
      position_root_to_end = Eigen::Vector3d(x, y - l1, z - l0);
    }
    else
    {
      position_root_to_end = Eigen::Vector3d(x, y + l1, z - l0);
    }

    // Calculate distance from shoulder to end effector
    double distance_to_end = position_root_to_end.norm();
    
    // Check if target is reachable
    if (distance_to_end > (l2 + l3) || distance_to_end < fabs(l2 - l3))
    {
      // Target is out of reach
      return {false, joint_angles};
    }

    //-----------------------------------------------------------------------------------------------------------
    // Calculate joint 1 (shoulder yaw/base rotation)
    theta[k_joint_1] = atan2(position_root_to_end.y(), position_root_to_end.x());
    
    // Project to shoulder frame
    double r = sqrt(position_root_to_end.x() * position_root_to_end.x() + 
                   position_root_to_end.y() * position_root_to_end.y());
    double s = position_root_to_end.z();
    
    // Calculate joint 4 (elbow) using cosine rule
    double cos_elbow = (l2 * l2 + l3 * l3 - distance_to_end * distance_to_end) / (2 * l2 * l3);
    
    // Clamp to valid range
    if (cos_elbow > 1.0) cos_elbow = 1.0;
    if (cos_elbow < -1.0) cos_elbow = -1.0;
    
    theta[k_joint_4] = M_PI - acos(cos_elbow);
    
    // Calculate joint 2 (shoulder pitch)
    double alpha = atan2(s, r);
    double beta = acos((l2 * l2 + distance_to_end * distance_to_end - l3 * l3) / 
                      (2 * l2 * distance_to_end));
    
    if (is_left_arm)
    {
      theta[k_joint_2] = alpha + beta;
    }
    else
    {
      theta[k_joint_2] = alpha + beta;
    }
    
    // For 4-DOF arm, joint 3 (shoulder roll) is often used for orientation
    // This is a simplified approach - for full orientation control, 
    // you would need to solve the orientation constraints
    theta[k_joint_3] = 0.0; // Can be set based on roll requirement
    
    //-----------------------------------------------------------------------------------------------------------
    // Check for NaN values
    bool ik_result = true;
    for (int i = 0; i < 4; i++)
    {
      if (std::isnan(theta[i]))
      {
        ik_result = false;
        break;
      }
      
      // Clean up small values
      if (abs(theta[i]) < k_epsilon)
        theta[i] = 0;
        
      joint_angles[i] = aeirobot::NormalizeRadian(theta[i]);
    }
    
    return {ik_result, joint_angles};
  }

  aeirobot_msgs::msg::PoseXYZRPY FourDofArmKinematics::CalcForwardKinematics(const std::vector<double> &joint_angles)
  {
    if (joint_angles.size() != 4)
    {
      throw std::invalid_argument("Joint angles vector must have 4 elements.");
    }

    double theta1 = joint_angles[0];
    double theta2 = joint_angles[1]; 
    double theta3 = joint_angles[2];
    double theta4 = joint_angles[3];

    // Forward kinematics using DH parameters for 4-DOF arm
    Eigen::Matrix4d transform_mat_world_to_base;
    Eigen::Matrix4d transform_mat_base_to_joint1;
    Eigen::Matrix4d transform_mat_joint1_to_joint2;
    Eigen::Matrix4d transform_mat_joint2_to_joint3;
    Eigen::Matrix4d transform_mat_joint3_to_joint4;
    Eigen::Matrix4d transform_mat_joint4_to_end;

    if (is_left_arm)
    {
      // Left arm transformation matrices
      transform_mat_world_to_base = aeirobot::DHMatrix(0, 0, l0, 0);
      transform_mat_base_to_joint1 = aeirobot::DHMatrix(0, 0, l1, 0);
      transform_mat_joint1_to_joint2 = aeirobot::DHMatrix(M_PI/2, 0.0, 0.0, theta1);
      transform_mat_joint2_to_joint3 = aeirobot::DHMatrix(0, l2, 0.0, theta2);
      transform_mat_joint3_to_joint4 = aeirobot::DHMatrix(M_PI/2, 0.0, 0.0, theta3);
      transform_mat_joint4_to_end = aeirobot::DHMatrix(0, l3, 0.0, theta4);
    }
    else
    {
      // Right arm transformation matrices  
      transform_mat_world_to_base = aeirobot::DHMatrix(0, 0, l0, 0);
      transform_mat_base_to_joint1 = aeirobot::DHMatrix(0, 0, -l1, 0);
      transform_mat_joint1_to_joint2 = aeirobot::DHMatrix(M_PI/2, 0.0, 0.0, theta1);
      transform_mat_joint2_to_joint3 = aeirobot::DHMatrix(0, l2, 0.0, theta2);
      transform_mat_joint3_to_joint4 = aeirobot::DHMatrix(M_PI/2, 0.0, 0.0, theta3);
      transform_mat_joint4_to_end = aeirobot::DHMatrix(0, l3, 0.0, theta4);
    }

    // Calculate total transformation matrix
    Eigen::Matrix4d transform_mat_world_to_end = transform_mat_world_to_base * 
                                                 transform_mat_base_to_joint1 *
                                                 transform_mat_joint1_to_joint2 * 
                                                 transform_mat_joint2_to_joint3 *
                                                 transform_mat_joint3_to_joint4 * 
                                                 transform_mat_joint4_to_end;

    // Extract position and orientation
    Eigen::Vector3d position = transform_mat_world_to_end.block<3, 1>(0, 3);
    Eigen::Matrix3d rotation = transform_mat_world_to_end.block<3, 3>(0, 0);

    // Clean up very small values
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

    // Extract Euler angles (ZYX order)
    Eigen::Vector3d euler_angles = rotation.eulerAngles(2, 1, 0);

    aeirobot_msgs::msg::PoseXYZRPY pose;
    pose.x = position(0);
    pose.y = position(1);
    pose.z = position(2);
    pose.roll = euler_angles(2);   // Roll
    pose.pitch = euler_angles(1);  // Pitch  
    pose.yaw = euler_angles(0);    // Yaw

    return pose;
  }
}