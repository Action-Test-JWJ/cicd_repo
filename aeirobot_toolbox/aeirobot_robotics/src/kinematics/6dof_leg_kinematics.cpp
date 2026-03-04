#include "aeirobot_robotics/kinematics/6dof_leg_kinematics.h"

/*********************************************************
  _       _____    ____
 | |     | ____|  / ___|
 | |     |  _|   | |  _
 | |___  | |___  | |_| |
 |_____| |_____|  \____|

*********************************************************/
namespace aeirobot
{
  KinematicsDynamics::KinematicsDynamics()
  {
    // ReadKinematicsYaml();

    // for (int id = 0; id <= ALL_JOINT_ID; id++)
    //   alice4_link_data_[id] = new LinkData();

    // for (int link_id = 0; link_id < ALL_JOINT_ID; link_id++)
    // {
    //   YAML::Node joint_node = kinematics_doc_[kine_link_name_[link_id]];

    //   double jlmax = joint_node["joint_limit_max"].as<double>();
    //   double jlmin = joint_node["joint_limit_min"].as<double>();
    //   if (jlmax > -50 && jlmax < 50)
    //     jlmax = jlmax * M_PI;
    //   if (jlmin > -50 && jlmin < 50)
    //     jlmin = jlmin * M_PI;

    //   std::vector<double> rp = joint_node["relative_position"].as<std::vector<double>>();
    //   std::vector<double> ja = joint_node["joint_axis"].as<std::vector<double>>();
    //   std::vector<double> cm = joint_node["center_of_mass"].as<std::vector<double>>();
    //   std::vector<double> ia = joint_node["inertia"].as<std::vector<double>>();

    //   alice4_link_data_[link_id]->name_ = kine_link_name_[link_id];
    //   alice4_link_data_[link_id]->parent_ = joint_node["parent"].as<double>();
    //   alice4_link_data_[link_id]->sibling_ = joint_node["sibling"].as<double>();
    //   alice4_link_data_[link_id]->child_ = joint_node["child"].as<double>();
    //   alice4_link_data_[link_id]->mass_ = joint_node["mass"].as<double>();
    //   alice4_link_data_[link_id]->relative_position_ = aeirobot::getTransitionXYZ(rp[0], rp[1], rp[2]);
    //   alice4_link_data_[link_id]->joint_axis_ = aeirobot::getTransitionXYZ(ja[0], ja[1], ja[2]);
    //   alice4_link_data_[link_id]->center_of_mass_ = aeirobot::getTransitionXYZ(cm[0], cm[1], cm[2]);
    //   alice4_link_data_[link_id]->joint_limit_max_ = jlmax;
    //   alice4_link_data_[link_id]->joint_limit_min_ = jlmin;
    //   alice4_link_data_[link_id]->inertia_ = aeirobot::getInertiaXYZ(ia[0], ia[1], ia[2], ia[3], ia[4], ia[5]); //(ixx, ixy, ixz, iyy, iyz, izz)
    // }
    thigh_length_m_ = aeirobot::GetParameter<double>("walking_parameters", "thigh_length_m");
    calf_length_m_ = aeirobot::GetParameter<double>("walking_parameters", "calf_length_m");
    ankle_length_m_ = aeirobot::GetParameter<double>("walking_parameters", "ankle_length_m");

    KinematicsGraig();
    alice4_version = aeirobot::GetParameter<int>("alice4_version", "version_name");
    LoadRobotModel();
  }
  KinematicsDynamics::~KinematicsDynamics()
  {
  }

  void KinematicsDynamics::ReadKinematicsYaml()
  {
    std::string kinematics_path = ament_index_cpp::get_package_share_directory("alice4_parameters") + "/config/kinematics_dynamics.yaml"; // AB param yaml

    try
    {
      kinematics_doc_ = YAML::LoadFile(kinematics_path.c_str());
    }
    catch (const std::exception &e)
    {
      ROS_RED_STREAM("Fail to load kinematics yaml file!");
      return;
    }
    // ROS_RED_STREAM("printed_kinematics yaml data!");
    // ROS_RED_STREAM("%s",kinematics_path.c_str());

    std::string temp_kine_link_name[ALL_JOINT_ID] = {
        "base",
        // motor 1-8 꼭 로봇파일에 있는 ID랑 순서가 같아야할지 모르겠지만 보기 좋게 순서를 배열했음.BNCH
        "l_arm_sh_p", "r_arm_sh_p", "l_arm_sh_r", "r_arm_sh_r", "l_arm_sh_y", "r_arm_sh_y",
        "l_arm_el_p", "r_arm_el_p",
        // motor 9-12
        "head_p", "head_y", "waist_y", "waist_p",
        // motor 13-24
        "l_leg_hip_p", "r_leg_hip_p", "l_leg_hip_r", "r_leg_hip_r", "l_leg_hip_y", "r_leg_hip_y", // PRY yaml file is kin_dyn_3 copy.yaml
        "l_leg_kn_p", "r_leg_kn_p", "l_leg_an_p", "r_leg_an_p", "l_leg_an_r", "r_leg_an_r",
        // end point 25-28
        "l_arm_end", "r_arm_end", "l_leg_end", "r_leg_end",
        // other 29-36
        "cam", "pelvis",
        "passive_x", "passive_y", "passive_z", "passive_yaw", "passive_pitch", "passive_roll"};

    for (int t = 0; t < ALL_JOINT_ID; t++)
      kine_link_name_[t] = temp_kine_link_name[t];
  }

  void KinematicsDynamics::CalcForwardKinematics(int joint_id)
  {
    if (joint_id == -1)
      return;

    if (joint_id == 0)
    {
      // alice4_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
      alice4_link_data_[0]->position_ = Eigen::Matrix<double, 3, 1>::Zero();
      alice4_link_data_[0]->orientation_ =
          aeirobot::calcRodrigues(aeirobot::calcHatto(alice4_link_data_[0]->joint_axis_), alice4_link_data_[0]->joint_angle_);
    }

    if (joint_id != 0)
    {
      int parent = alice4_link_data_[joint_id]->parent_;

      alice4_link_data_[joint_id]->position_ =
          alice4_link_data_[parent]->orientation_ * alice4_link_data_[joint_id]->relative_position_ + alice4_link_data_[parent]->position_;
      alice4_link_data_[joint_id]->orientation_ =
          alice4_link_data_[parent]->orientation_ *
          aeirobot::calcRodrigues(aeirobot::calcHatto(alice4_link_data_[joint_id]->joint_axis_), alice4_link_data_[joint_id]->joint_angle_);

      // alice4_link_data_[joint_id]->transformation_.block<3,1>(0,3) = alice4_link_data_[joint_id]->position_;
      // alice4_link_data_[joint_id]->transformation_.block<3,3>(0,0) = alice4_link_data_[joint_id]->orientation_;
    }

    CalcForwardKinematics(alice4_link_data_[joint_id]->sibling_);
    CalcForwardKinematics(alice4_link_data_[joint_id]->child_);
  }

  bool KinematicsDynamics::ComputeInverseKinematics(double *out, double x, double y, double z, double roll, double pitch, double yaw, [[maybe_unused]] bool is_left)
  {
    // std::cout << "des : ";
    // std::cout << std::fixed << std::setprecision(3) <<x<<" "<<y<<" "<<z<<" "<<roll<<" "<<pitch<<" "<<yaw<<"\n";

    if (alice4_version == 1)
    {
      Eigen::Matrix4d trans_ad;
      z += 0.055;
      if (is_left)
      {
        y -= 0.11;
        trans_ad = aeirobot::GetTransformationMatrix(x, y, z, roll, pitch, yaw);
        trans_ad = trans_ad * aeirobot::GetTransformationMatrix(0, 0, 0, aeirobot::DegToRad(-5.0), 0, 0);
      }
      else
      {
        y += 0.11;
        trans_ad = aeirobot::GetTransformationMatrix(x, y, z, roll, pitch, yaw);
        trans_ad = trans_ad * aeirobot::GetTransformationMatrix(0, 0, 0, aeirobot::DegToRad(5.0), 0, 0);
      }
      Eigen::Vector3d end_effector_pos = trans_ad.block<3, 1>(0, 3);

      // yaw_error 만큼 반대로 회전하는 회전 변환 행렬을 생성
      Eigen::Matrix4d yaw_rotation_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, 0, -yaw);

      // 끝단 위치에 동차 좌표계를 사용하여 변환 행렬을 적용
      Eigen::Vector4d hom_end_effector_pos;
      hom_end_effector_pos << end_effector_pos, 1.0; // 동차 좌표계로 변환 (4차원으로 확장)

      Eigen::Vector3d rotated_end_effector_pos = (yaw_rotation_matrix * hom_end_effector_pos).block<3, 1>(0, 0); // 회전 후의 끝단 위치
                                                                                                                 // 1. hip_roll 계산 (rotated_end_effector_pos의 y, z 좌표 사용)
      double hip_roll = std::atan2(rotated_end_effector_pos.z(), rotated_end_effector_pos.y());

      // 각도가 음수일 경우 π/2를 더해 양수로 변환
      if (hip_roll < 0)
      {
        hip_roll += M_PI / 2;
      }
      // Hip Roll 회전 변환 행렬 생성 (hip_roll만 적용된 변환 행렬)
      Eigen::Matrix4d hip_roll_rotation = aeirobot::GetTransformationMatrix(0, 0, 0, hip_roll, 0, 0);
      // Hip Roll 회전 후 Z축으로 0.055만큼 이동
      Eigen::Vector4d hip_pitch_offset(0, 0, -0.055, 1);                    // Z축 방향으로 -0.055 이동 (동차 좌표계)
      Eigen::Vector4d hip_pitch_pos = hip_roll_rotation * hip_pitch_offset; // 회전된 위치에서 offset 적용

      // 엔드포인트까지의 벡터 계산 (rotated_end_effector_pos는 3D, hip_pitch_pos는 동차 좌표계에서 변환된 3D 위치)
      Eigen::Vector3d delta_pos = rotated_end_effector_pos - hip_pitch_pos.block<3, 1>(0, 0); // hip_pitch_pos의 상위 3D 좌표를 추출

      // Hip Pitch 계산 (XZ 평면에서의 각도)
      double hip_pitch = std::atan2(delta_pos.z(), delta_pos.x()); // XZ 평면에서의 각도 계산
      hip_pitch += M_PI / 2;                                       // 각도를 0에서 2π로 변환
      hip_pitch = hip_pitch * -1;                                  // 각도 반전
      // 허벅지와 종아리 길이
      double thigh_length = 0.355;
      double shin_length = 0.355;

      // delta_pos 벡터의 길이 계산 (distance_to_endpoint)
      double distance_to_endpoint = delta_pos.norm(); // Eigen의 norm() 함수로 벡터 길이 계산
      // std::cout << "distance_to_endpoint : " << distance_to_endpoint << std::endl;
      // 코사인 법칙을 이용한 무릎 각도 계산
      double knee_angle;
      // (distance_to_endpoint**2 - thigh_length**2 - shin_length**2) / (2.0 * thigh_length * shin_length)
      double cos_angle = ((distance_to_endpoint * distance_to_endpoint) - (thigh_length * thigh_length) - (shin_length * shin_length)) / (2.0 * thigh_length * shin_length);
      // std::cout << "cos_angle : " << cos_angle << std::endl;
      if (cos_angle >= -1.0 && cos_angle <= 1.0)
      {
        knee_angle = std::acos(cos_angle); // 무릎 각도 계산
      }
      else
      {
        // 예외 처리: 잘못된 입력일 경우 오류 반환
        std::cerr << "무릎 각도 계산 오류: 입력 파라미터를 확인하세요" << std::endl;
        return false;
      }

      hip_pitch -= knee_angle / 2.0;
      // Hip Pitch에서 무릎 각도의 절반을 뺌

      // 무릎에서 발목까지의 벡터 계산 (종아리 길이 벡터)
      Eigen::Vector3d knee_to_ankle_vec(0, 0, -shin_length); // 종아리 길이 벡터

      // Hip Roll 회전 적용
      Eigen::Matrix3d hip_roll_rotation_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, hip_roll, 0, 0).block<3, 3>(0, 0);
      knee_to_ankle_vec = hip_roll_rotation_matrix * knee_to_ankle_vec; // Hip Roll 적용

      // Hip Pitch 회전 적용
      Eigen::Matrix3d hip_pitch_rotation_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, hip_pitch, 0).block<3, 3>(0, 0);
      knee_to_ankle_vec = hip_pitch_rotation_matrix * knee_to_ankle_vec; // Hip Pitch 적용

      // Knee Pitch 회전 적용
      Eigen::Matrix3d knee_pitch_rotation_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, knee_angle, 0).block<3, 3>(0, 0);
      knee_to_ankle_vec = knee_pitch_rotation_matrix * knee_to_ankle_vec; // Knee Pitch 적용

      // Ankle Roll 적용 및 Z축으로 -0.015m 이동
      Eigen::Vector3d ankle_offset(0, 0, -0.015);                                // Z축으로 -0.015 이동
      Eigen::Vector3d ankle_pos_after_offset = knee_to_ankle_vec + ankle_offset; // 이동 후 위치

      // Ankle Roll 적용
      double ankle_roll = -hip_roll + roll;
      // // if(!is_left)
      // // {
      // //   ankle_roll *=-1;
      // //   // hip_pitch += knee_angle / 2.0;
      // // }
      Eigen::Matrix3d ankle_roll_rotation_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, ankle_roll, 0, 0).block<3, 3>(0, 0);
      Eigen::Vector3d ankle_pos_final = ankle_roll_rotation_matrix * ankle_pos_after_offset; // Ankle Roll 적용 후 위치

      // // Ankle Pitch 계산
      double ankle_pitch = std::atan2(ankle_pos_final.z(), ankle_pos_final.x()) + M_PI / 2 - pitch;

      // Hip Yaw 변환 행렬 (기본 yaw 회전)
      Eigen::Matrix4d hip_yaw_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, 0, yaw);

      // Hip Roll 변환 행렬 (hip_roll 적용)
      Eigen::Matrix4d hip_roll_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, hip_roll, 0, 0);

      // Hip Roll 후 Z축으로 -0.055m 이동
      Eigen::Matrix4d hip_offset_matrix = aeirobot::GetTransformationMatrix(0, 0, -0.055, 0, 0, 0);

      // Hip Pitch 변환 행렬 (hip_pitch 적용)
      Eigen::Matrix4d hip_pitch_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, hip_pitch, 0);

      // Hip Pitch 후 Z축으로 -0.355m 이동한 곳에 Knee Pitch 적용
      Eigen::Matrix4d knee_offset_matrix = aeirobot::GetTransformationMatrix(0, 0, -0.355, 0, 0, 0);
      Eigen::Matrix4d knee_pitch_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, knee_angle, 0);

      // Knee Pitch 후 Z축으로 -0.355m 이동한 곳에 Ankle Roll 적용
      Eigen::Matrix4d ankle_roll_offset_matrix = aeirobot::GetTransformationMatrix(0, 0, -0.355, 0, 0, 0);
      Eigen::Matrix4d ankle_roll_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, ankle_roll, 0, 0);

      // Ankle Roll 후 Z축으로 -0.015m 이동한 곳에 Ankle Pitch 적용
      Eigen::Matrix4d ankle_pitch_offset_matrix = aeirobot::GetTransformationMatrix(0, 0, -0.015, 0, 0, 0);
      Eigen::Matrix4d ankle_pitch_matrix = aeirobot::GetTransformationMatrix(0, 0, 0, 0, ankle_pitch, 0);

      // 최종 변환 행렬 계산 (각각의 변환 행렬을 순차적으로 곱함)
      Eigen::Matrix4d actual_transformation_matrix = hip_yaw_matrix *
                                                     hip_roll_matrix *
                                                     hip_offset_matrix *
                                                     hip_pitch_matrix *
                                                     knee_offset_matrix *
                                                     knee_pitch_matrix *
                                                     ankle_roll_offset_matrix *
                                                     ankle_roll_matrix *
                                                     ankle_pitch_offset_matrix *
                                                     ankle_pitch_matrix;

      // 실제 변환 행렬과 목표 변환 행렬의 차이를 구함
      Eigen::Matrix4d error_transformation_matrix = actual_transformation_matrix.inverse() * trans_ad;

      // 회전 행렬을 추출하고 오일러 각도로 변환
      aeirobot_msgs::msg::PoseXYZRPY error_pose = aeirobot::GetPose3DfromTransformMatrix(error_transformation_matrix);

      // 회전 각도 오차 적용
      // 오차를 현재 각도에 반영
      hip_pitch += error_pose.pitch;

      hip_roll += error_pose.roll;
      yaw += error_pose.yaw;
      *(out) = hip_pitch;
      *(out + 1) = hip_roll;
      *(out + 2) = yaw;
      *(out + 3) = knee_angle;
      *(out + 4) = ankle_pitch;
      *(out + 5) = ankle_roll;

      // Eigen::VectorXd calc_joint(6);
      // calc_joint << hip_pitch, hip_roll, yaw, knee_angle, ankle_pitch, ankle_roll;

      return true;
    }
    else if (alice4_version == 2)
    {
      Eigen::Matrix4d trans_ad;
      if (is_left)
      {
        // ROS_GREEN_STREAM("LEFT");
        y -= 0.105;
      }
      else
      {
        // ROS_GREEN_STREAM("RIGHT");
        y += 0.105;
      }
      trans_ad = aeirobot::GetTransformationMatrix(x, y, z, roll, pitch, yaw);
      Eigen::Matrix4d trans_da, trans_cd, trans_dc;
      Eigen::Matrix3d rot_ac;
      Eigen::Vector3d vec;

      bool invertible;
      double rac, arc_cos, arc_tan, alpha;
      double thigh_length = 0.37;
      double calf_length = 0.37;
      double ankle_length = 0.049;

      vec.coeffRef(0) = trans_ad.coeff(0, 3) + trans_ad.coeff(0, 2) * ankle_length;
      vec.coeffRef(1) = trans_ad.coeff(1, 3) + trans_ad.coeff(1, 2) * ankle_length;
      vec.coeffRef(2) = trans_ad.coeff(2, 3) + trans_ad.coeff(2, 2) * ankle_length;

      // Get Knee
      rac = vec.norm();
      arc_cos = acos(
          (rac * rac - thigh_length * thigh_length - calf_length * calf_length) / (2.0 * thigh_length * calf_length));
      if (std::isnan(arc_cos) == 1)
        return false;
      *(out + 3) = arc_cos;

      // Get Ankle Roll
      trans_ad.computeInverseWithCheck(trans_da, invertible);
      if (invertible == false)
        return false;

      vec.coeffRef(0) = trans_da.coeff(0, 3);
      vec.coeffRef(1) = trans_da.coeff(1, 3);
      vec.coeffRef(2) = trans_da.coeff(2, 3) - ankle_length;

      arc_tan = atan2(vec(1), vec(2));
      if (arc_tan > M_PI_2)
        arc_tan = arc_tan - M_PI;
      else if (arc_tan < -M_PI_2)
        arc_tan = arc_tan + M_PI;

      *(out + 5) = arc_tan;

      // Get Ankle Pitch
      alpha = asin(thigh_length * sin(M_PI - *(out + 3)) / rac);
      *(out + 4) = -atan2(vec(0), copysign(sqrt(vec(1) * vec(1) + vec(2) * vec(2)), vec(2))) - alpha;

      // Get Hip Pitch
      // rot_ac = (aeirobot::convertRPYToRotation(roll, pitch, yaw) * aeirobot::getRotationX(-(*(out + 5)))) * aeirobot::getRotationY(-(*(out + 3) + *(out + 4)));
      rot_ac = aeirobot::getRotationZ(yaw) *
               aeirobot::getRotationY(pitch) *
               aeirobot::getRotationX(roll) *
               aeirobot::getRotationX(-(*(out + 5))) *
               aeirobot::getRotationY(-(*(out + 3) + *(out + 4)));

      arc_tan = atan2(rot_ac.coeff(0, 2), rot_ac.coeff(2, 2));
      *(out) = arc_tan;

      // Get Hip Roll

      arc_tan = atan2(-rot_ac.coeff(1, 2), rot_ac.coeff(0, 2) * sin(*(out)) + rot_ac.coeff(2, 2) * cos(*(out)));
      *(out + 1) = arc_tan;

      // yaw축이 pitch 방향으로  -30도만큼 돌아 있기때문에 yaw 회전이 roll과 pitch 각도에 영향을 주게됨

      // Get Hip Yaw
      Eigen::Matrix3d R_dec = aeirobot::getRotationX(-(*(out + 1))) *                        // -hip_roll
                              aeirobot::getRotationY(-(*(out)) + aeirobot::DegToRad(30.0)) * // -hip_pitch
                              aeirobot::getRotationX((*(out + 5))) *
                              aeirobot::getRotationY(-(*(out + 4))) *
                              rot_ac;
      arc_tan = atan2(R_dec.coeff(1, 0), R_dec.coeff(1, 1));
      *(out + 2) = arc_tan;

      *(out + 1) = *(out + 1) * (1.0 - std::sin(*(out) + aeirobot::DegToRad(30.0)));
      *(out + 5) = *(out + 5) * (1.0 - std::sin(*(out) + aeirobot::DegToRad(30.0)));

      return true;
    }
    else
    {
      std::cerr << "알수없는 alice4 version" << std::endl;
      return false;
    }
  }

  bool KinematicsDynamics::ComputeInverseKinematicsForLeftLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
  {
    // 왼다리 관절 순서 (IK 결과 out의 순서)
    std::vector<std::string> left_leg_joints = {
        "l_hip_p", "l_hip_r", "l_hip_y",
        "l_knee_p", "l_ankle_p", "l_ankle_r"};

    // 로봇 모델 내 관절 순서 (q 벡터의 인덱스 순서)
    std::vector<std::string> left_model_leg_joints = {
        "l_hip_y", "l_hip_r", "l_hip_p",
        "l_knee_p", "l_ankle_r", "l_ankle_p"};

    // 기존 ComputeInverseKinematics 함수를 호출하여 out 배열 갱신
    ComputeInverseKinematics(out, x, y, z, roll, pitch, yaw, true);

    // ------------------------------------------------------------------
    // Step 1. Iterative IK 방식 시도 (프레임 "l_leg_end_joint" 사용, 왼쪽 다리)
    // ------------------------------------------------------------------
    // 목표 pose 구성 (입력된 위치와 오일러 각을 회전행렬로 변환)
    Eigen::Matrix3d R = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
    pinocchio::SE3 oMdes(R, Eigen::Vector3d(x, y, z));

    // 로봇 모델의 중립 상태로 q 초기화 (free-flyer 포함)
    Eigen::VectorXd q = pinocchio::neutral(robot_model);

    // left_leg_joints (IK 결과 순서) → left_model_leg_joints (모델 순서) 매핑하여 q 업데이트
    for (size_t i = 0; i < left_model_leg_joints.size(); i++)
    {
      // left_model_leg_joints의 관절 이름을 left_leg_joints에서 검색
      auto it = std::find(left_leg_joints.begin(), left_leg_joints.end(), left_model_leg_joints[i]);
      if (it != left_leg_joints.end())
      {
        size_t out_index = std::distance(left_leg_joints.begin(), it);
        // 로봇 모델에서 해당 관절의 인덱스 획득
        pinocchio::JointIndex jid = robot_model.getJointId(left_model_leg_joints[i]);
        int config_index = robot_model.joints[jid].idx_q();
        // out 벡터의 값을 q에 대입
        q[config_index] = out[out_index];
        // ROS_CYAN_STREAM(std::fixed << std::setprecision(3)
        //                            << left_model_leg_joints[i] << " (from iterative IK) : " << out[out_index]);
      }
    }

    // Iterative IK 파라미터
    const double eps = 1e-4;
    const int IT_MAX = 10;
    const double DT = 1.0;
    const double damp = 1e-4; // 수치적 안정성 보완을 위해 약간의 덤핑

    pinocchio::Data::Matrix6x J(6, robot_model.nv);
    J.setZero();

    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model.nv);
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;

    // "l_leg_end_joint" 프레임의 id 획득
    pinocchio::FrameIndex frame_id = robot_model.getFrameId("l_leg_end_joint");

    // 람다 함수 정의: 지정한 프레임의 pose 정보를 출력 (label 포함)
    // auto PrintFrameInfo = [&](pinocchio::FrameIndex f_id, const std::string &label)
    // {
    //   const pinocchio::SE3 &final_pose = robot_data.oMf[f_id];
    //   const Eigen::Vector3d &position = final_pose.translation();
    //   Eigen::Vector3d rpy = final_pose.rotation().eulerAngles(2, 1, 0); // 0: roll, 1: pitch, 2: yaw
    //   ROS_BLUE_STREAM(std::fixed << std::setprecision(3)
    //                              << label << " | X : " << position.x()
    //                              << " Y : " << position.y()
    //                              << " Z : " << position.z()
    //                              << " Roll : " << aeirobot::RadToDeg(rpy(0))
    //                              << " Pitch : " << aeirobot::RadToDeg(rpy(1))
    //                              << " Yaw : " << aeirobot::RadToDeg(rpy(2)));
    // };
    // PrintFrameInfo(frame_id, "LEFT LEG END FRAME FK");
    bool iterative_success = false;

    pinocchio::Data::Matrix6x Jlog;
    Eigen::Matrix<double, 6, 6> JJt;
    Jlog.resize(6, 6);
    Jlog.setZero();
    JJt.setZero();
    for (int i = 0;; i++)
    {
      // 현재 q에 대해 FK 계산 및 프레임 배치 갱신 (oMf 업데이트)
      pinocchio::forwardKinematics(robot_model, robot_data, q);
      pinocchio::updateFramePlacement(robot_model, robot_data, frame_id);

      const pinocchio::SE3 current_pose = robot_data.oMf[frame_id];
      const pinocchio::SE3 diff = current_pose.actInv(oMdes);
      err = pinocchio::log6(diff).toVector();

      if (err.norm() < eps)
      {
        iterative_success = true;
        break;
      }
      if (i >= IT_MAX)
      {
        iterative_success = false;
        break;
      }

      // 프레임 잭코비안 계산 (LOCAL 좌표 기준)
      pinocchio::computeFrameJacobian(robot_model, robot_data, q, frame_id, pinocchio::LOCAL, J);

      // 오차 보정을 위한 Jlog6 계산
      // pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(diff.inverse(), Jlog);
      J = -Jlog * J;

      // damped least-squares 방법으로 속도 업데이트
      JJt = J * J.transpose();
      JJt.diagonal().array() += damp;
      v = -J.transpose() * JJt.ldlt().solve(err);

      // q 업데이트 (integrate 사용)
      q = pinocchio::integrate(robot_model, q, v * DT);

      // if ((i % 1) == 0)
      // {
      //   std::cout << i << ": left error = " << err.transpose() << std::endl;
      // }
    }

    if (iterative_success)
    {
      // std::cout << "Iterative IK converged!" << std::endl;

      // 최종 q로 FK 및 프레임 배치 갱신
      // pinocchio::forwardKinematics(robot_model, robot_data, q);
      // pinocchio::updateFramePlacements(robot_model, robot_data);

      // 람다 함수로 "l_leg_end_joint" 프레임의 pose 정보 출력
      // PrintFrameInfo(frame_id, "LEFT LEG END FRAME FK");

      for (size_t i = 0; i < left_leg_joints.size(); i++)
      {
        pinocchio::JointIndex jid = robot_model.getJointId(left_leg_joints[i]);
        int config_index = robot_model.joints[jid].idx_q(); // 각 관절이 1 자유도라고 가정
        out[i] = q[config_index];
        // ROS_CYAN_STREAM(std::fixed << std::setprecision(3)
        //                            << left_leg_joints[i] << " (iterative IK) : " <<  aeirobot::RadToDeg(out[i]));
      }
      return true;
    }
    else
    {
      // ------------------------------------------------------------------
      // Step 2. Iterative IK가 수렴하지 않으면 기존 방식 사용
      // ------------------------------------------------------------------
      // ROS_RED_STREAM("Iterative IK failed. Using original IK method.");
      // pinocchio::FrameIndex frame_id_fallback = robot_model.getFrameId("l_leg_end_joint");
      // PrintFrameInfo(frame_id_fallback, "LEFT LEG END FRAME FK");
      bool original_success = ComputeInverseKinematics(out, x, y, z, roll, pitch, yaw, true);
      return original_success;
    }
  }

  bool KinematicsDynamics::ComputeInverseKinematicsForRightLeg(double *out, double x, double y, double z, double roll, double pitch, double yaw)
  {
    // 왼다리 관절 순서 (IK 결과 out의 순서)
    std::vector<std::string> right_leg_joints = {
        "r_hip_p", "r_hip_r", "r_hip_y",
        "r_knee_p", "r_ankle_p", "r_ankle_r"};

    // 로봇 모델 내 관절 순서 (q 벡터의 인덱스 순서)
    std::vector<std::string> right_model_leg_joints = {
        "r_hip_y", "r_hip_r", "r_hip_p",
        "r_knee_p", "r_ankle_r", "r_ankle_p"};

    // 기존 ComputeInverseKinematics 함수를 호출하여 out 배열 갱신
    ComputeInverseKinematics(out, x, y, z, roll, pitch, yaw, false);

    // ------------------------------------------------------------------
    // Step 1. Iterative IK 방식 시도 (프레임 "l_leg_end_joint" 사용, 왼쪽 다리)
    // ------------------------------------------------------------------
    // 목표 pose 구성 (입력된 위치와 오일러 각을 회전행렬로 변환)
    Eigen::Matrix3d R = (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
                            .toRotationMatrix();
    pinocchio::SE3 oMdes(R, Eigen::Vector3d(x, y, z));

    // 로봇 모델의 중립 상태로 q 초기화 (free-flyer 포함)
    Eigen::VectorXd q = pinocchio::neutral(robot_model);

    // right_leg_joints (IK 결과 순서) → right_model_leg_joints (모델 순서) 매핑하여 q 업데이트
    for (size_t i = 0; i < right_model_leg_joints.size(); i++)
    {
      // right_model_leg_joints의 관절 이름을 right_leg_joints에서 검색
      auto it = std::find(right_leg_joints.begin(), right_leg_joints.end(), right_model_leg_joints[i]);
      if (it != right_leg_joints.end())
      {
        size_t out_index = std::distance(right_leg_joints.begin(), it);
        // 로봇 모델에서 해당 관절의 인덱스 획득
        pinocchio::JointIndex jid = robot_model.getJointId(right_model_leg_joints[i]);
        int config_index = robot_model.joints[jid].idx_q();
        // out 벡터의 값을 q에 대입
        q[config_index] = out[out_index];
        // ROS_CYAN_STREAM(std::fixed << std::setprecision(3)
        //                            << right_model_leg_joints[i] << " (from iterative IK) : " << out[out_index]);
      }
    }

    // Iterative IK 파라미터
    const double eps = 1e-4;
    const int IT_MAX = 10;
    const double DT = 1.0;
    const double damp = 1e-4; // 수치적 안정성 보완을 위해 약간의 덤핑

    pinocchio::Data::Matrix6x J(6, robot_model.nv);
    J.setZero();

    Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model.nv);
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    Vector6d err;

    // "l_leg_end_joint" 프레임의 id 획득
    pinocchio::FrameIndex frame_id = robot_model.getFrameId("r_leg_end_joint");

    // 람다 함수 정의: 지정한 프레임의 pose 정보를 출력 (label 포함)
    // auto PrintFrameInfo = [&](pinocchio::FrameIndex f_id, const std::string &label)
    // {
    //   const pinocchio::SE3 &final_pose = robot_data.oMf[f_id];
    //   const Eigen::Vector3d &position = final_pose.translation();
    //   Eigen::Vector3d rpy = final_pose.rotation().eulerAngles(2, 1, 0); // 0: roll, 1: pitch, 2: yaw
    //   ROS_BLUE_STREAM(std::fixed << std::setprecision(3)
    //                              << label << " | X : " << position.x()
    //                              << " Y : " << position.y()
    //                              << " Z : " << position.z()
    //                              << " Roll : " << aeirobot::RadToDeg(rpy(0))
    //                              << " Pitch : " << aeirobot::RadToDeg(rpy(1))
    //                              << " Yaw : " << aeirobot::RadToDeg(rpy(2)));
    // };
    // PrintFrameInfo(frame_id, "RIGHT LEG END FRAME FK");
    bool iterative_success = false;
    pinocchio::Data::Matrix6x Jlog;
    Eigen::Matrix<double, 6, 6> JJt;
    Jlog.resize(6, 6);
    Jlog.setZero();
    JJt.setZero();
    for (int i = 0;; i++)
    {
      // 현재 q에 대해 FK 계산 및 프레임 배치 갱신 (oMf 업데이트)
      pinocchio::forwardKinematics(robot_model, robot_data, q);
      pinocchio::updateFramePlacement(robot_model, robot_data, frame_id);

      const pinocchio::SE3 current_pose = robot_data.oMf[frame_id];
      const pinocchio::SE3 diff = current_pose.actInv(oMdes);
      err = pinocchio::log6(diff).toVector();

      if (err.norm() < eps)
      {
        iterative_success = true;
        break;
      }
      if (i >= IT_MAX)
      {
        iterative_success = false;
        break;
      }

      // 프레임 잭코비안 계산 (LOCAL 좌표 기준)
      pinocchio::computeFrameJacobian(robot_model, robot_data, q, frame_id, pinocchio::LOCAL, J);

      // 오차 보정을 위한 Jlog6 계산
      // pinocchio::Data::Matrix6 Jlog;
      pinocchio::Jlog6(diff.inverse(), Jlog);
      J = -Jlog * J;

      // damped least-squares 방법으로 속도 업데이트
      JJt = J * J.transpose();
      JJt.diagonal().array() += damp;
      v = -J.transpose() * JJt.ldlt().solve(err);

      // q 업데이트 (integrate 사용)
      q = pinocchio::integrate(robot_model, q, v * DT);

      // if ((i % 1) == 0)
      // {
      //   std::cout << i << ": right error = " << err.transpose() << std::endl;
      // }
    }

    if (iterative_success)
    {
      // std::cout << "Iterative IK converged!" << std::endl;

      // 최종 q로 FK 및 프레임 배치 갱신
      // pinocchio::forwardKinematics(robot_model, robot_data, q);
      // pinocchio::updateFramePlacements(robot_model, robot_data);

      // 람다 함수로 "r_leg_end_joint" 프레임의 pose 정보 출력
      // PrintFrameInfo(frame_id, "right LEG END FRAME FK");

      for (size_t i = 0; i < right_leg_joints.size(); i++)
      {
        pinocchio::JointIndex jid = robot_model.getJointId(right_leg_joints[i]);
        int config_index = robot_model.joints[jid].idx_q(); // 각 관절이 1 자유도라고 가정
        out[i] = q[config_index];
        // ROS_CYAN_STREAM(std::fixed << std::setprecision(3)
        //                            << right_leg_joints[i] << " (iterative IK) : " << aeirobot::RadToDeg(out[i]));
      }
      return true;
    }
    else
    {
      // ------------------------------------------------------------------
      // Step 2. Iterative IK가 수렴하지 않으면 기존 방식 사용
      // ------------------------------------------------------------------
      // ROS_RED_STREAM("Iterative IK failed. Using original IK method.");
      // 여기서는 "r_leg_end_joint" 프레임을 사용하여 fallback 결과를 확인
      // pinocchio::FrameIndex frame_id_fallback = robot_model.getFrameId("r_leg_end_joint");
      // PrintFrameInfo(frame_id_fallback, "RIGHT LEG END FRAME FK");
      bool original_success = ComputeInverseKinematics(out, x, y, z, roll, pitch, yaw, false);
      return original_success;
    }
  }

  bool KinematicsDynamics::KinematicsGraig()
  {
    joint_radian.resize(7, 1);
    joint_radian.fill(0);

    // kinematics variables //
    P_inverse_.fill(0);
    P_.fill(0);

    // DH convention variables
    dh_alpha[0] = 0;
    dh_alpha[1] = M_PI / 2;
    dh_alpha[2] = -M_PI / 2;
    dh_alpha[3] = -M_PI / 2;
    dh_alpha[4] = 0;
    dh_alpha[5] = M_PI / 2;
    dh_alpha[6] = 0;

    dh_link[0] = 0;
    dh_link[1] = 0;
    dh_link[2] = 0;
    dh_link[3] = 0;
    dh_link[4] = 0.240;
    dh_link[5] = 0;
    dh_link[6] = 0.127;

    total_length_ = 0.607;

    dh_link_d[0] = 0;
    dh_link_d[1] = 0;
    dh_link_d[2] = 0;
    dh_link_d[3] = 0.240;
    dh_link_d[4] = 0;
    dh_link_d[5] = 0;
    dh_link_d[6] = 0;

    real_theta[0] = 0;
    real_theta[1] = 0;
    real_theta[2] = 0;
    real_theta[3] = 0;
    real_theta[4] = 0;
    real_theta[5] = 0;
    real_theta[6] = 0;

    for (int i = 0; i < 8; i++)
    {
      // H[i].resize(4, 4);
      // H[i].fill(0);
      H[i].setZero();
    }

    // center_to_sensor_transform_right.resize(4, 4);
    // center_to_sensor_transform_right.fill(0);
    // center_to_sensor_transform_left.resize(4, 4);
    // center_to_sensor_transform_left.fill(0);
    // center_to_foot_transform_left_leg.resize(4, 4);
    // center_to_foot_transform_left_leg.fill(0);
    // center_to_foot_transform_right_leg.resize(4, 4);
    // center_to_foot_transform_right_leg.fill(0);
    // H_ground_to_center.resize(4, 4);
    // H_ground_to_center.fill(0);

    center_to_sensor_transform_right.setZero();
    center_to_sensor_transform_left.setZero();
    center_to_foot_transform_left_leg.setZero();
    center_to_foot_transform_right_leg.setZero();
    H_ground_to_center.setZero();

    H[7] << 0, 0, -1, 0,
        0, 1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1;

    H_ground_to_center << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, total_length_,
        0, 0, 0, 1;

    return 0;
  }

  void KinematicsDynamics::LoadRobotModel()
  {
    std::string urdf_path;
    if (aeirobot::GetParameter<int>("alice4_version", "version_name") == 1)
    {
      urdf_path = ament_index_cpp::get_package_share_directory("alice4_description") + "/urdf/" + "alice4.1.urdf";
    }
    else if (aeirobot::GetParameter<int>("alice4_version", "version_name") == 2)
    {
      urdf_path = ament_index_cpp::get_package_share_directory("alice4_description") + "/urdf/" + "alice4.2.urdf";
    }
    else
    {
      ROS_RED_STREAM("Invalid ALICE Version!!!");
    }

    // URDF를 Pinocchio 모델로 변환
    try
    {
      // pinocchio::JointModelFreeFlyer jointModelFF;
      // pinocchio::urdf::buildModel(urdf_path, jointModelFF, robot_model);
      pinocchio::urdf::buildModel(urdf_path, robot_model);
      robot_data = pinocchio::Data(robot_model);
      ROS_GREEN_STREAM("KD Successfully loaded URDF model.");
      ROS_GREEN_STREAM("KD URDF model address : " << urdf_path);
    }
    catch (const std::exception &e)
    {
      ROS_RED_STREAM("KD Failed to load URDF model: " << e.what());
    }
  }
}
