#ifndef GAIT_CONTROLLER_HPP
#define GAIT_CONTROLLER_HPP

#include "alfan_kinematics/gait_generator.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <eigen3/Eigen/Dense>

struct WalkControl {
    std::vector<Eigen::Vector2d> com_position_fix;
    std::vector<Eigen::Vector2d> com_velocity_fix;
    std::vector<Eigen::Vector2d> zmp_fix;
};

class GaitController {
    public:
        // Constructor
        GaitController();

        // Destructor
        ~GaitController();

        // Control gait algorithm
        WalkControl controlGait(const WalkParameter& planned_gait);
    private:
        // Parameter node and client (currently unused)
        rclcpp::Node::SharedPtr parameter_node_;
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;
};

#endif // GAIT_CONTROLLER_HPP
