#include "kinematics.hpp"
#include <iostream>

// Constructor
KinematicsSolver::KinematicsSolver(double thigh_length, double shin_length, double foot_height)
  : thigh_(thigh_length), shin_(shin_length), foot_height_(foot_height) {
}

Eigen::Vector3d KinematicsSolver::computeFK(const Eigen::VectorXd& joint_angles, bool is_right_leg) {
  // Joint angles: [hip_roll, hip_pitch, hip_yaw, knee_pitch, ankle_pitch, ankle_roll]
  double hip_roll = joint_angles(0);
  double hip_pitch = joint_angles(1);
  double hip_yaw = joint_angles(2);
  double knee_pitch = joint_angles(3);
  double ankle_pitch = joint_angles(4);
  double ankle_roll = joint_angles(5);

  // Compute transformation matrices for each joint
  Eigen::Matrix4d T_hip_roll = dhMatrix(hip_roll, 0, 0, EIGEN_PI / 2);
  Eigen::Matrix4d T_hip_pitch = dhMatrix(hip_pitch, 0, 0, -EIGEN_PI / 2);
  Eigen::Matrix4d T_hip_yaw = dhMatrix(hip_yaw, 0, 0, EIGEN_PI / 2);
  Eigen::Matrix4d T_knee = dhMatrix(knee_pitch, thigh_, 0, 0);
  Eigen::Matrix4d T_ankle_pitch = dhMatrix(ankle_pitch, shin_, 0, 0);
  Eigen::Matrix4d T_ankle_roll = dhMatrix(ankle_roll, 0, foot_height_, 0);

  // Combine transformations
  Eigen::Matrix4d T_foot = T_hip_roll * T_hip_pitch * T_hip_yaw * T_knee * T_ankle_pitch * T_ankle_roll;

  // Extract foot position (x, y, z)
  Eigen::Vector3d foot_pos(T_foot(0, 3), T_foot(1, 3), T_foot(2, 3));
  return foot_pos;
}

Eigen::VectorXd KinematicsSolver::computeIK(const Eigen::Vector3d& foot_pos, bool is_right_leg) {
  // Hip position relative to torso (assume torso is at origin)
  double hip_offset_y = is_right_leg ? -0.044 : 0.044; // 5 cm offset for left/right legs
  Eigen::Vector3d hip_pos(0, hip_offset_y, thigh_ + shin_ + foot_height_);

  // Vector from hip to foot
  Eigen::Vector3d hip_to_foot = foot_pos - hip_pos;

  // Solve for hip roll (rotation around X-axis)
  double hip_roll = atan2(hip_to_foot.y(), hip_to_foot.z());

  // Solve for hip pitch and knee pitch
  double L = hip_to_foot.norm(); // Distance from hip to foot
  double hip_pitch = -acos((thigh_ * thigh_ + L * L - shin_ * shin_) / (2 * thigh_ * L)) + atan2(hip_to_foot.x(), hip_to_foot.z());
  double knee_pitch = acos((thigh_ * thigh_ + shin_ * shin_ - L * L) / (2 * thigh_ * shin_)) - EIGEN_PI;

  // Solve for ankle pitch and roll
  double ankle_pitch = -hip_pitch - knee_pitch;
  double ankle_roll = -hip_roll;

  // Return joint angles [hip_roll, hip_pitch, hip_yaw, knee_pitch, ankle_pitch, ankle_roll]
  Eigen::VectorXd joint_angles(6);
  joint_angles << hip_roll, hip_pitch, 0, knee_pitch, ankle_pitch, ankle_roll;
  return joint_angles;
}

Eigen::Matrix4d KinematicsSolver::dhMatrix(double theta, double a, double d, double alpha) {
  Eigen::Matrix4d T;
  T << cos(theta), -sin(theta) * cos(alpha), sin(theta)* sin(alpha), a* cos(theta),
    sin(theta), cos(theta)* cos(alpha), -cos(theta) * sin(alpha), a* sin(theta),
    0, sin(alpha), cos(alpha), d,
    0, 0, 0, 1;
  return T;
}