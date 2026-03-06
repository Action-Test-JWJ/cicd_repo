#include "aeirobot_math/algebra/linear_algebra.hpp"

namespace aeirobot
{

  Eigen::Vector3d getTransitionXYZ(double position_x, double position_y, double position_z)
  {
    Eigen::Vector3d position;

    position << position_x,
        position_y,
        position_z;

    return position;
  }

  Eigen::Matrix4d getTransformationXYZRPY(double position_x, double position_y, double position_z, double roll, double pitch, double yaw)
  {
    Eigen::Matrix4d transformation = getRotation4d(roll, pitch, yaw);
    transformation.coeffRef(0, 3) = position_x;
    transformation.coeffRef(1, 3) = position_y;
    transformation.coeffRef(2, 3) = position_z;

    return transformation;
  }

  Eigen::Matrix4d getTransformationXYZRPY(Pose6D pose)
  {
    return getTransformationXYZRPY(pose.position.x, pose.position.y, pose.position.z,
                                    pose.angle.roll, pose.angle.pitch, pose.angle.yaw);
  }

  Eigen::Matrix4d getInverseTransformation(const Eigen::Matrix4d &transform)
  {
    // If T is Transform Matrix A from B, the BOA is translation component coordi. B to coordi. A

    Eigen::Vector3d vec_boa;
    Eigen::Vector3d vec_x, vec_y, vec_z;
    Eigen::Matrix4d inv_t;

    vec_boa(0) = -transform(0, 3);
    vec_boa(1) = -transform(1, 3);
    vec_boa(2) = -transform(2, 3);

    vec_x(0) = transform(0, 0);
    vec_x(1) = transform(1, 0);
    vec_x(2) = transform(2, 0);
    vec_y(0) = transform(0, 1);
    vec_y(1) = transform(1, 1);
    vec_y(2) = transform(2, 1);
    vec_z(0) = transform(0, 2);
    vec_z(1) = transform(1, 2);
    vec_z(2) = transform(2, 2);

    inv_t << vec_x(0), vec_x(1), vec_x(2), vec_boa.dot(vec_x),
        vec_y(0), vec_y(1), vec_y(2), vec_boa.dot(vec_y),
        vec_z(0), vec_z(1), vec_z(2), vec_boa.dot(vec_z),
        0, 0, 0, 1;

    return inv_t;
  }

  Eigen::Matrix3d getInertiaXYZ(double ixx, double ixy, double ixz, double iyy, double iyz, double izz)
  {
    Eigen::Matrix3d inertia;

    inertia << ixx, ixy, ixz,
        ixy, iyy, iyz,
        ixz, iyz, izz;

    return inertia;
  }

  Eigen::Matrix3d getRotationX(double angle)
  {
    Eigen::Matrix3d rotation(3, 3);

    rotation << 1.0, 0.0, 0.0,
        0.0, cos(angle), -sin(angle),
        0.0, sin(angle), cos(angle);

    return rotation;
  }

  Eigen::Matrix3d getRotationY(double angle)
  {
    Eigen::Matrix3d rotation(3, 3);

    rotation << cos(angle), 0.0, sin(angle),
        0.0, 1.0, 0.0,
        -sin(angle), 0.0, cos(angle);

    return rotation;
  }

  Eigen::Matrix3d getRotationZ(double angle)
  {
    Eigen::Matrix3d rotation(3, 3);

    rotation << cos(angle), -sin(angle), 0.0,
        sin(angle), cos(angle), 0.0,
        0.0, 0.0, 1.0;

    return rotation;
  }

  Eigen::Matrix4d getRotation4d(double roll, double pitch, double yaw)
  {
    double sr = sin(roll), cr = cos(roll);
    double sp = sin(pitch), cp = cos(pitch);
    double sy = sin(yaw), cy = cos(yaw);

    Eigen::Matrix4d mat_roll;
    Eigen::Matrix4d mat_pitch;
    Eigen::Matrix4d mat_yaw;

    mat_roll << 1, 0, 0, 0,
        0, cr, -sr, 0,
        0, sr, cr, 0,
        0, 0, 0, 1;

    mat_pitch << cp, 0, sp, 0,
        0, 1, 0, 0,
        -sp, 0, cp, 0,
        0, 0, 0, 1;

    mat_yaw << cy, -sy, 0, 0,
        sy, cy, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Matrix4d mat_rpy = (mat_yaw * mat_pitch) * mat_roll;

    return mat_rpy;
  }

  Eigen::Matrix4d getTranslation4D(double position_x, double position_y, double position_z)
  {
    Eigen::Matrix4d mat_translation;

    mat_translation << 1, 0, 0, position_x,
        0, 1, 0, position_y,
        0, 0, 1, position_z,
        0, 0, 0, 1;

    return mat_translation;
  }

  Eigen::Vector3d convertRotationToRPY(const Eigen::Matrix3d &rotation)
  {
    Eigen::Vector3d rpy; // = Eigen::MatrixXd::Zero(3,1);

    rpy.coeffRef(0, 0) = atan2(rotation.coeff(2, 1), rotation.coeff(2, 2));
    rpy.coeffRef(1, 0) = atan2(-rotation.coeff(2, 0), sqrt(pow(rotation.coeff(2, 1), 2) + pow(rotation.coeff(2, 2), 2)));
    rpy.coeffRef(2, 0) = atan2(rotation.coeff(1, 0), rotation.coeff(0, 0));

    return rpy;
  }

  Eigen::Matrix3d convertRPYToRotation(double roll, double pitch, double yaw)
  {
    Eigen::Matrix3d rotation = getRotationZ(yaw) * getRotationY(pitch) * getRotationX(roll);

    return rotation;
  }

  Eigen::Quaterniond convertRPYToQuaternion(double roll, double pitch, double yaw)
  {
    Eigen::Quaterniond quaternion;
    quaternion = convertRPYToRotation(roll, pitch, yaw);

    return quaternion;
  }

  Eigen::Quaterniond convertRotationToQuaternion(const Eigen::Matrix3d &rotation)
  {
    Eigen::Quaterniond quaternion;
    quaternion = rotation;

    return quaternion;
  }

  Eigen::Vector3d convertQuaternionToRPY(const Eigen::Quaterniond &quaternion)
  {
    Eigen::Vector3d rpy = convertRotationToRPY(quaternion.toRotationMatrix());

    return rpy;
  }

  Eigen::Matrix3d convertQuaternionToRotation(const Eigen::Quaterniond &quaternion)
  {
    Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

    return rotation;
  }

  Eigen::Matrix3d calcHatto(const Eigen::Vector3d &matrix3d)
  {
    // Eigen::MatrixXd hatto(3, 3);
    Eigen::Matrix3d hatto;

    hatto << 0.0, -matrix3d.coeff(2, 0), matrix3d.coeff(1, 0),
        matrix3d.coeff(2, 0), 0.0, -matrix3d.coeff(0, 0),
        -matrix3d.coeff(1, 0), matrix3d.coeff(0, 0), 0.0;

    return hatto;
  }

  Eigen::Matrix3d calcRodrigues(const Eigen::Matrix3d &hat_matrix, double angle)
  {
    //  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(3,3);
    //  Eigen::MatrixXd rodrigues = identity+hat_matrix*sin(angle)+hat_matrix*hat_matrix*(1-cos(angle));
    Eigen::Matrix3d rodrigues = hat_matrix * sin(angle) + hat_matrix * hat_matrix * (1 - cos(angle));
    rodrigues.coeffRef(0, 0) += 1;
    rodrigues.coeffRef(1, 1) += 1;
    rodrigues.coeffRef(2, 2) += 1;

    return rodrigues;
  }

  Eigen::Vector3d convertRotToOmega(const Eigen::Matrix3d &rotation)
  {
    double eps = 1e-10;

    double alpha = (rotation.coeff(0, 0) + rotation.coeff(1, 1) + rotation.coeff(2, 2) - 1.0) / 2.0;
    double alpha_dash = fabs(alpha - 1.0);

    Eigen::Vector3d rot_to_omega;

    if (alpha_dash < eps)
    {
      rot_to_omega << 0.0,
          0.0,
          0.0;
    }
    else
    {
      double theta = acos(alpha);

      rot_to_omega << rotation.coeff(2, 1) - rotation.coeff(1, 2),
          rotation.coeff(0, 2) - rotation.coeff(2, 0),
          rotation.coeff(1, 0) - rotation.coeff(0, 1);

      rot_to_omega = 0.5 * (theta / sin(theta)) * rot_to_omega;
    }

    return rot_to_omega;
  }

  Eigen::Vector3d calcCross(const Eigen::Vector3d &vector3d_a, const Eigen::Vector3d &vector3d_b)
  {
    Eigen::Vector3d cross;

    cross << vector3d_a.coeff(1, 0) * vector3d_b.coeff(2, 0) - vector3d_a.coeff(2, 0) * vector3d_b.coeff(1, 0),
        vector3d_a.coeff(2, 0) * vector3d_b.coeff(0, 0) - vector3d_a.coeff(0, 0) * vector3d_b.coeff(2, 0),
        vector3d_a.coeff(0, 0) * vector3d_b.coeff(1, 0) - vector3d_a.coeff(1, 0) * vector3d_b.coeff(0, 0);

    return cross;
  }

  double calcInner(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
  {
    return a.dot(b);
  }

  Pose3D getPose3DfromTransformMatrix(const Eigen::Matrix4d &transform)
  {
    Pose3D pose_3d;

    pose_3d.x = transform.coeff(0, 3);
    pose_3d.y = transform.coeff(1, 3);
    pose_3d.z = transform.coeff(2, 3);
    pose_3d.roll = atan2(transform.coeff(2, 1), transform.coeff(2, 2));
    pose_3d.pitch = atan2(-transform.coeff(2, 0), sqrt(transform.coeff(2, 1) * transform.coeff(2, 1) + transform.coeff(2, 2) * transform.coeff(2, 2)));
    pose_3d.yaw = atan2(transform.coeff(1, 0), transform.coeff(0, 0));

    return pose_3d;
  }

  //////////////////////////

  aeirobot_msgs::msg::PoseXYZRPY GetPose3DfromTransformMatrix(const Eigen::Matrix4d &transform)
  {
    aeirobot_msgs::msg::PoseXYZRPY pose_3d;

    pose_3d.x = transform.coeff(0, 3);
    pose_3d.y = transform.coeff(1, 3);
    pose_3d.z = transform.coeff(2, 3);
    pose_3d.roll = atan2(transform.coeff(2, 1), transform.coeff(2, 2));
    pose_3d.pitch = atan2(-transform.coeff(2, 0), sqrt(transform.coeff(2, 1) * transform.coeff(2, 1) + transform.coeff(2, 2) * transform.coeff(2, 2)));
    pose_3d.yaw = atan2(transform.coeff(1, 0), transform.coeff(0, 0));

    return pose_3d;
  }

  Eigen::Vector3d RotationMatrixToRPY(const Eigen::Matrix3d &rotationMatrix)
  {
    Eigen::Vector3d rpy;

    double sy = std::sqrt(rotationMatrix(0, 0) * rotationMatrix(0, 0) + rotationMatrix(1, 0) * rotationMatrix(1, 0));

    bool singular = sy < 1e-6; // 임계값으로 수평 상태 판정

    if (!singular)
    {
      rpy[0] = std::atan2(rotationMatrix(2, 1), rotationMatrix(2, 2)); // roll
      rpy[1] = std::atan2(-rotationMatrix(2, 0), sy);                  // pitch
      rpy[2] = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0)); // yaw
    }
    else
    {
      rpy[0] = std::atan2(-rotationMatrix(1, 2), rotationMatrix(1, 1)); // roll
      rpy[1] = std::atan2(-rotationMatrix(2, 0), sy);                   // pitch
      rpy[2] = 0;                                                       // yaw (불명확)
    }

    return rpy;
  }

  Eigen::Matrix4d GetTransformationMatrix(const double x, double y, double z, double roll, double pitch, double yaw)
  {
    // 위치 변환 행렬
    Eigen::Matrix4d translation = Eigen::Matrix4d::Identity();
    translation(0, 3) = x;
    translation(1, 3) = y;
    translation(2, 3) = z;

    // 회전 행렬 (Z-Y-X 순서로 Roll-Pitch-Yaw 적용)
    Eigen::Matrix3d rotation;

    double cosRoll = std::cos(roll);
    double sinRoll = std::sin(roll);
    double cosPitch = std::cos(pitch);
    double sinPitch = std::sin(pitch);
    double cosYaw = std::cos(yaw);
    double sinYaw = std::sin(yaw);

    rotation << cosYaw * cosPitch, cosYaw * sinPitch * sinRoll - sinYaw * cosRoll, cosYaw * sinPitch * cosRoll + sinYaw * sinRoll,
        sinYaw * cosPitch, sinYaw * sinPitch * sinRoll + cosYaw * cosRoll, sinYaw * sinPitch * cosRoll - cosYaw * sinRoll,
        -sinPitch, cosPitch * sinRoll, cosPitch * cosRoll;

    // 4x4 회전 변환 행렬
    Eigen::Matrix4d rotation4x4 = Eigen::Matrix4d::Identity();
    rotation4x4.block<3, 3>(0, 0) = rotation;

    // 최종 변환 행렬 (회전 + 평행 이동)
    Eigen::Matrix4d transformation = translation * rotation4x4;

    return transformation;
  }

  Eigen::Matrix3d SkewSymmetricMatrix(const Eigen::Vector3d &v)
  {
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(),
        v.z(), 0, -v.x(),
        -v.y(), v.x(), 0;
    return skew;
  }

  Eigen::Matrix4d DHMatrix(double alpha, double a, double d, double theta)
  {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;
    return T;
  }

  Eigen::Matrix4d ModifiedDHMatrix(double alpha, double a, double d, double theta)
  {
    Eigen::Matrix4d T;
    T << cos(theta), -sin(theta), 0, a,
        sin(theta) * cos(alpha), cos(theta) * cos(alpha), -sin(alpha), -sin(alpha) * d,
        sin(theta) * sin(alpha), cos(theta) * sin(alpha), cos(alpha), cos(alpha) * d,
        0, 0, 0, 1;
    return T;
  }

  // Eigen::MatrixXd DampedLeastSquaresInverse(const Eigen::MatrixXd &J, double epsilon = 1e-6)
  Eigen::Matrix<double, 6, Eigen::Dynamic> DampedLeastSquaresInverse(const Eigen::Matrix<double, Eigen::Dynamic, 6> &J, 
                                                                     double epsilon = 1e-6)
  {
    int m = J.rows(); // Eigen::Dynamic
    int n = J.cols(); // 6
    if (m >= n)
    {
      // 왼쪽 역행렬 (m >= n 인 경우)
      // return (J.transpose() * J + epsilon * Eigen::MatrixXd::Identity(n, n)).inverse() * J.transpose();
      return (J.transpose() * J + epsilon * Eigen::Matrix<double, 6, 6>::Identity()).inverse() * J.transpose();
    }
    else
    {
      // 오른쪽 역행렬 (m < n 인 경우)
      // return J.transpose() * (J * J.transpose() + epsilon * Eigen::MatrixXd::Identity(m, m)).inverse();
      return J.transpose() * (J * J.transpose() + epsilon * Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(m, m)).inverse();
    }
  }

  Eigen::Matrix4d PoseToTransformMatrix(const geometry_msgs::msg::Pose & pose)
  {
      // Orientation (Quaternion)
      tf2::Quaternion q(
          pose.orientation.x,
          pose.orientation.y,
          pose.orientation.z,
          pose.orientation.w
      );
    
      tf2::Matrix3x3 rot(q);  // Rotation matrix
      Eigen::Matrix3d R;
      for (int i = 0; i < 3; i++)
          for (int j = 0; j < 3; j++)
              R(i, j) = rot[i][j];
    
      // Translation
      Eigen::Vector3d t(
          pose.position.x,
          pose.position.y,
          pose.position.z
      );
    
      // Homogeneous Transformation Matrix
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.block<3,3>(0,0) = R;
      T.block<3,1>(0,3) = t;
    
      return T;
  }
}
