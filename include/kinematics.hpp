#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "eigen-3.4.0/Eigen/Dense"
#include <cmath>

class KinematicsSolver {
public:
  KinematicsSolver(double thigh_length, double shin_length, double foot_height);

  /**
   * @brief Forward Kinematics: Compute foot position from joint angles
   *
   * @param joint_angles Vector besar sudut-sudut putar servo (θ)
   * @param is_right_leg True untuk kaki kanan, False untuk kaki kiri
   * @return Eigen::Vector3d - Vector posisi akhir kaki tersebut
   */
  Eigen::Vector3d computeFK(const Eigen::VectorXd& joint_angles, bool is_right_leg);

  /**
   * @brief Inverse Kinematics: Compute joint angles from foot position
   *
   * @param foot_pos Vector posisi foot yang diinginkan
   * @param is_right_leg Kaki kanan(True), kiri (False)
   * @return Eigen::VectorXd - Vector yang isinya joint angle/besar sudut putar servo (θ)
   */
  Eigen::VectorXd computeIK(const Eigen::Vector3d& foot_pos, bool is_right_leg);

private:
  double thigh_;       // Length of the thigh (hip to knee) in meter
  double shin_;        // Length of the shin (knee to ankle) in meter
  double foot_height_; // Height of the foot (ankle to foot sole) in meter

  /**
   * @brief Helper function to compute the transformation matrix using DH parameters
   *
   * @param theta
   * @param a
   * @param d
   * @param alpha
   * @return Eigen::Matrix4d of DH Parameter for each leg
   */
  Eigen::Matrix4d dhMatrix(double theta, double a, double d, double alpha);
};

#endif // KINEMATICS_HPP