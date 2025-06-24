#include "alfan_kinematics/alfan_kinematics.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

KinematicsSolver::KinematicsSolver() {
    uLINK = std::vector<Link>(15);
    initLinks(uLINK);
}

KinematicsSolver::~KinematicsSolver() {}

void KinematicsSolver::computeFK(int j = 1) {
    if (j == 0)
        return;

    // For non-root links, update the position and orientation based on the parent's state.
    if (j != 1) {
        int parent = uLINK[j].mother;
        uLINK[j].p = uLINK[parent].R * uLINK[j].b + uLINK[parent].p;
        uLINK[j].R = uLINK[parent].R * rodrigues(uLINK[j].a, uLINK[j].q);
    }

    // Recursively compute forward kinematics for sister and child links.
    computeFK(uLINK[j].sister);
    computeFK(uLINK[j].child);
}

Eigen::VectorXd KinematicsSolver::computeIK(std::vector<double> legPosition, Leg leg) {
    const Link& body = uLINK[1];
    double A = THIGH_LENGTH;
    double B = SHIN_LENGTH;

    double x = legPosition[0];
    double y = legPosition[1];
    double z = legPosition[2];

    setFootPos(Eigen::Vector3d(x, y, z), leg);

    Link foot;
    double D = 0.0;
    if (leg == Leg::LEFT) {
        foot = uLINK[7];
        D = HIP_OFFSET_Y;
    } else { // Leg::RIGHT
        foot = uLINK[13];
        D = -HIP_OFFSET_Y;
    }

    Eigen::Vector3d r = foot.R.transpose() * (body.p + body.R * Eigen::Vector3d(0, D, 0) - foot.p);
    double C = r.norm();

    // Compute the knee joint angle using the cosine law.
    double c5 = (C * C - A * A - B * B) / (2.0 * A * B);
    c5 = std::clamp(c5, -1.0, 1.0);  // Clamp to [-1, 1] for numerical safety.
    double q5 = acos(c5);

    double q6a = asin((A / C) * sin(M_PI - q5));
    double q7 = atan2(r.y(), r.z());
    if (q7 > M_PI / 2)
        q7 -= M_PI;
    else if (q7 < -M_PI / 2)
        q7 += M_PI;
    double q6 = -atan2(r.x(), copysign(sqrt(r.y() * r.y() + r.z() * r.z()), r.z())) - q6a;

    // Compute orientation differences for the hip joints.
    Eigen::Matrix3d R = body.R.transpose() * foot.R *
                        rodrigues(Eigen::Vector3d::UnitX(), -q7) *
                        rodrigues(Eigen::Vector3d::UnitY(), -q6 - q5);
    double q2 = atan2(-R(0, 1), R(1, 1));
    double cz = cos(q2), sz = sin(q2);
    double q3 = atan2(R(2, 1), -R(0, 1) * sz + R(1, 1) * cz);
    double q4 = atan2(-R(2, 0), R(2, 2));


    double hip_yaw      = q2;
    double hip_roll     = q3;
    double hip_pitch    = q4;
    double knee_pitch   = q5;
    double ankle_pitch  = q6;
    double ankle_roll   = q7;

    Eigen::VectorXd angles(6);
    angles << hip_yaw, hip_roll, hip_pitch, knee_pitch, ankle_pitch, ankle_roll;

    setJointAngles(angles, leg);
    return angles;
}

void KinematicsSolver::setJointAngles(const Eigen::VectorXd& jointAngles, Leg leg) {
    if (jointAngles.size() != 6) {
        throw std::invalid_argument("Error: Expected 6 joint angles.");
    }
    if (leg == Leg::LEFT) {
        for (int i = 2; i <= 7; ++i) {
            uLINK[i].q = jointAngles(i - 2);
        }
    } else { // Leg::RIGHT
        for (int i = 8; i <= 13; ++i) {
            uLINK[i].q = jointAngles(i - 8);
        }
    }
}

void KinematicsSolver::setFootPos(const Eigen::Vector3d desiredFootPos, Leg leg) {
    if (leg == Leg::LEFT) {
        Eigen::Vector3d groundFootPosition (0, HIP_OFFSET_Y, 0);
        uLINK[7].p = groundFootPosition + desiredFootPos;

    } else { // Leg::RIGHT
        Eigen::Vector3d groundFootPosition (0, -HIP_OFFSET_Y, 0);
        uLINK[13].p = groundFootPosition + desiredFootPos;
    }
}

Eigen::Matrix3d KinematicsSolver::rodrigues(const Eigen::Vector3d& w, double theta) {
    double norm_w = w.norm();
    if (norm_w < 1e-6)
        return Eigen::Matrix3d::Identity();

    Eigen::Vector3d w_normalized = w.normalized();
    Eigen::Matrix3d skew;
    skew << 0, -w_normalized.z(), w_normalized.y(),
            w_normalized.z(), 0, -w_normalized.x(),
           -w_normalized.y(), w_normalized.x(), 0;

    return Eigen::Matrix3d::Identity() +
           sin(theta) * skew +
           (1 - cos(theta)) * (skew * skew);
}

void KinematicsSolver::initLinks(std::vector<Link>& uLINK) {    
    // Initialize the body link.
    // The body link is the root of the kinematic chain and does not have a parent.
    // It has a child link (LHIP_YAW) and no sister link.
    // The position is set to the hip height, and the orientation is the identity matrix.
    // The joint axis is along the Z-axis.
    // The joint angle is initialized to 0.
    uLINK[1] = { "BODY", 0, 2, 0,
                 Eigen::Vector3d(0, 0, HIP_HEIGHT),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::UnitZ(),
                 0 };

    // Initialize the left leg links.
    // Each link in the left leg is connected in a chain from the hip to the ankle.
    // The position and orientation of each link are initialized based on the previous link.
    // The joint axes are set according to the type of joint (yaw, roll, pitch).
    // The joint angles are initialized to 0.
    uLINK[2] = { "LHIP_YAW", 8, 3, 1,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, HIP_HEIGHT),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(0, HIP_OFFSET_Y, 0),
                 Eigen::Vector3d::UnitZ(),
                 0 };
    uLINK[3] = { "LHIP_ROLL", 0, 4, 2,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, HIP_HEIGHT),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::UnitX(),
                 0 };
    uLINK[4] = { "LHIP_PITCH", 0, 5, 3,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, HIP_HEIGHT),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d::Zero(),
                 Eigen::Vector3d::UnitY(),
                 0 };
    uLINK[5] = { "LKNEE", 0, 6, 4,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, KNEE_HEIGHT),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(0, 0, -THIGH_LENGTH),
                 Eigen::Vector3d::UnitY(),
                 0 };
    uLINK[6] = { "LANKLE_PITCH", 0, 7, 5,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, 0),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(0, 0, -SHIN_LENGTH),
                 Eigen::Vector3d::UnitY(),
                 0 };
    uLINK[7] = { "LANKLE_ROLL", 0, 0, 6,
                 Eigen::Vector3d(0, HIP_OFFSET_Y, 0),
                 Eigen::Matrix3d::Identity(),
                 Eigen::Vector3d(0, 0, 0),
                 Eigen::Vector3d::UnitX(),
                 0 };

    // Initialize the right leg links.
    // Each link in the right leg is connected in a chain from the hip to the ankle.
    // The position and orientation of each link are initialized based on the previous link.
    // The joint axes are set according to the type of joint (yaw, roll, pitch).
    // The joint angles are initialized to 0.
    uLINK[8]  = { "RHIP_YAW", 0, 9, 1,
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, HIP_HEIGHT),
                  Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, 0),
                  Eigen::Vector3d::UnitZ(),
                  0 };
    uLINK[9]  = { "RHIP_ROLL", 0, 10, 8,
                    Eigen::Vector3d(0, -HIP_OFFSET_Y, HIP_HEIGHT),
                    Eigen::Matrix3d::Identity(),
                    Eigen::Vector3d::Zero(),
                    Eigen::Vector3d::UnitX(),
                    0 };
    uLINK[10] = { "RHIP_PITCH", 0, 11, 9,
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, HIP_HEIGHT),
                  Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d::Zero(),
                  Eigen::Vector3d::UnitY(),
                  0 };
    uLINK[11] = { "RKNEE", 0, 12, 10,
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, KNEE_HEIGHT),
                  Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d(0, 0, -THIGH_LENGTH),
                  Eigen::Vector3d::UnitY(),
                  0 };
    uLINK[12] = { "RANKLE_PITCH", 0, 13, 11,
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, 0),
                  Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d(0, 0, -SHIN_LENGTH),
                  Eigen::Vector3d::UnitY(),
                  0 };
    uLINK[13] = { "RANKLE_ROLL", 0, 0, 12,
                  Eigen::Vector3d(0, -HIP_OFFSET_Y, 0),
                  Eigen::Matrix3d::Identity(),
                  Eigen::Vector3d(0, 0, 0),
                  Eigen::Vector3d::UnitX(),
                  0 };
}

void KinematicsSolver::traverseTree(int id, int depth) {
    if (id == 0)
        return;
    std::cout << std::string(depth * 2, ' ') << "- " << uLINK[id].name << "\n";
    traverseTree(uLINK[id].child, depth + 1);
    traverseTree(uLINK[id].sister, depth);
}

void KinematicsSolver::printPositions(){
    std::cout << "\nEnd Effector (Foot) Positions:\n";
    std::cout << "LFOOT: " << uLINK[7].p.transpose() << "\n";
    std::cout << "RFOOT: " << uLINK[13].p.transpose() << "\n";
}