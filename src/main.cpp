#include <iostream>
#include "../include/kinematics.hpp"

int main(){
    KinematicsSolver ks(0.094, 0.093, 0.034);
    Eigen::Vector3d foot_pos(0,0,0.2);
    Eigen::VectorXd angles = ks.computeIK(foot_pos, true);
    Eigen::Vector3d real_pos = ks.computeFK(angles, true);

    std::cout << angles << std::endl;
    std::cout << real_pos << std::endl;
    
}