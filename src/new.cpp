#include <iostream>
#include <vector>
#include <string>
#include "eigen3/Eigen/Dense" // For matrix operations

using namespace Eigen;

struct Link {
  std::string name;
  int sister;
  int child;
  int mother;
  Vector3d p; // Position (in meter)
  Matrix3d R; // Rotation matrix
  Vector3d b; // Offset from parent (in meter)
  Vector3d a; // Joint axis
  double q;   // Joint angle (in radian)
};

// Define global vector to hold the humanoid structure
std::vector<Link> uLINK(19); // 1-based index, ignoring index 0

Matrix3d Rodrigues(Vector3d w, double theta) {
  double norm_w = w.norm();
  if (norm_w < 1e-6) return Matrix3d::Identity();
  w.normalize();
  Matrix3d W;
  W << 0, -w.z(), w.y(),
    w.z(), 0, -w.x(),
    -w.y(), w.x(), 0;
  return Matrix3d::Identity() + sin(theta) * W + (1 - cos(theta)) * W * W;
}

void initializeHumanoid() {
  uLINK[1] = { "BODY",          0, 2, 0,  Vector3d(0, 0, 0.221),     Matrix3d::Identity(), Vector3d::Zero(),          Vector3d::UnitZ(), 0 };

  // Left Leg
  uLINK[2] = { "LHIP_YAW",      8, 3, 1,  Vector3d(0, 0.044, 0.221), Matrix3d::Identity(), Vector3d(0, 0.044, 0),   Vector3d::UnitZ(), 0 };
  uLINK[3] = { "LHIP_ROLL",     0, 4, 2,  Vector3d(0, 0.044, 0.221), Matrix3d::Identity(), Vector3d(0, 0, 0),       Vector3d::UnitX(), 0 };
  uLINK[4] = { "LHIP_PITCH",    0, 5, 3,  Vector3d(0, 0.044, 0.221), Matrix3d::Identity(), Vector3d(0, 0, 0),       Vector3d::UnitY(), 0 };
  uLINK[5] = { "LKNEE",         0, 6, 4,  Vector3d(0, 0.044, 0.127), Matrix3d::Identity(), Vector3d(0, 0, -0.094),  Vector3d::UnitY(), 0 };
  uLINK[6] = { "LANKLE_PITCH",  0, 7, 5,  Vector3d(0, 0.044, 0.034), Matrix3d::Identity(), Vector3d(0, 0, -0.093),  Vector3d::UnitY(), 0 };
  uLINK[7] = { "LANKLE_ROLL",   0, 0, 6,  Vector3d(0, 0.044, 0.034 + 0.015), Matrix3d::Identity(), Vector3d(0, 0, -0.034),  Vector3d::UnitX(), 0 };

  // Right Leg
  uLINK[8] = { "RHIP_YAW",      0, 9, 1,    Vector3d(0, -0.044, 0.221), Matrix3d::Identity(), Vector3d(0, -0.044, 0),  Vector3d::UnitZ(), 0 };
  uLINK[9] = { "RHIP_ROLL",     0, 10, 8,   Vector3d(0, -0.044, 0.221), Matrix3d::Identity(), Vector3d(0, 0, 0),       Vector3d::UnitX(), 0 };
  uLINK[10] = { "RHIP_PITCH",   0, 11, 9,   Vector3d(0, -0.044, 0.221), Matrix3d::Identity(), Vector3d(0, 0, 0),       Vector3d::UnitY(), 0 };
  uLINK[11] = { "RKNEE",        0, 12, 10,  Vector3d(0, -0.044, 0.127), Matrix3d::Identity(), Vector3d(0, 0, -0.094),  Vector3d::UnitY(), 0 };
  uLINK[12] = { "RANKLE_PITCH", 0, 13, 11,  Vector3d(0, -0.044, 0.034), Matrix3d::Identity(), Vector3d(0, 0, -0.093),  Vector3d::UnitY(), 0 };
  uLINK[13] = { "RANKLE_ROLL",  0, 0, 12,   Vector3d(0, -0.044, 0.034), Matrix3d::Identity(), Vector3d(0, 0, -0.034),  Vector3d::UnitX(), 0 };
}

void ForwardKinematics(int j) {
  if (j == 0) return;

  if (j != 1) {
    int i = uLINK[j].mother;
    uLINK[j].p = uLINK[i].R * uLINK[j].b + uLINK[i].p;
    uLINK[j].R = uLINK[i].R * Rodrigues(uLINK[j].a, uLINK[j].q);
    // std::cout << "Position vector after joint " << j << ":\n";
    std::cout << uLINK[j].p << std::endl;
  }

  ForwardKinematics(uLINK[j].sister);
  ForwardKinematics(uLINK[j].child);
}

VectorXd IK_leg(Vector3d Body_p, Matrix3d Body_R, double D, double A, double B, Vector3d Foot_p, Matrix3d Foot_R) {
  Vector3d r = Foot_R.transpose() * (Body_p + Body_R * Vector3d(0, D, 0) - Foot_p);
  double C = r.norm();
  double c5 = (C * C - A * A - B * B) / (2.0 * A * B);
  double q5 = (c5 >= 1) ? 0.0 : (c5 <= -1) ? M_PI : acos(c5);
  double q6a = asin((A / C) * sin(M_PI - q5));
  double q7 = atan2(r.y(), r.z());
  if (q7 > M_PI / 2) q7 -= M_PI;
  else if (q7 < -M_PI / 2) q7 += M_PI;
  double q6 = -atan2(r.x(), copysign(sqrt(r.y() * r.y() + r.z() * r.z()), r.z())) - q6a;

  Matrix3d R = Body_R.transpose() * Foot_R * Rodrigues(Vector3d::UnitX(), -q7) * Rodrigues(Vector3d::UnitY(), -q6 - q5);
  double q2 = atan2(-R(0, 1), R(1, 1));
  double cz = cos(q2), sz = sin(q2);
  double q3 = atan2(R(2, 1), -R(0, 1) * sz + R(1, 1) * cz);
  double q4 = atan2(-R(2, 0), R(2, 2));

  // test
  uLINK[2].q = q2;
  uLINK[3].q = q3;
  uLINK[4].q = q4;
  uLINK[5].q = q5;
  uLINK[6].q = q6;
  uLINK[7].q = q7;
  return (VectorXd(6) << q2, q3, q4, q5, q6, q7).finished();
}

// Recursive function to traverse and print the tree
void traverseTree(int id, int depth = 0) {
  if (id == 0) return;

  std::cout << std::string(depth * 2, ' ') << "- " << uLINK[id].name << "\n";

  traverseTree(uLINK[id].child, depth + 1);
  traverseTree(uLINK[id].sister, depth);
}

void printFootPositions() {
  std::cout << "\nEnd Effector (Foot) Positions:\n";
  std::cout << "LFOOT: " << uLINK[7].p.transpose() << "\n";
  std::cout << "RFOOT: " << uLINK[13].p.transpose() << "\n";
}

int main() {
  initializeHumanoid();
  VectorXd q = IK_leg(uLINK[1].p, uLINK[1].R, 0.044, 0.094, 0.093, uLINK[7].p, uLINK[7].R);
  std::cout << "Computed IK joint angles: " << q.transpose() * 180 / M_PI << "\n";
  ForwardKinematics(1);

  std::cout << "Humanoid Structure:\n";
  traverseTree(1);

  printFootPositions();


  return 0;
}