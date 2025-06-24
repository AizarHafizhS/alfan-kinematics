#include "alfan_kinematics/foot_step_planner.hpp"
#include <iostream>

FootStepPlanner::FootStepPlanner()
    : parameter_node_(rclcpp::Node::make_shared("FSPNode")),
        parameter_client_(std::make_shared<rclcpp::SyncParametersClient>(parameter_node_, "ParameterServerNode"))
{
    RCLCPP_INFO(parameter_node_->get_logger(), "Start FOOT STEP PLANNER.");

    // TODO: Access parameter step length from parameter server and fill step_length_ when available.

    // For now, use this hardcoded step length instead
    step_length_ = {
    //    x ,    y
        {0.0,   0.0 },  // step 0
        {0.0,   0.044},
        {0.03, -0.044},
        {0.03,  0.044},
        {0.03, -0.044},
        {0.03,  0.044},
        {0.0,   0.0},
        {0.0,   0.0}    // step 7
    };
    // Calculate initial foot positions
    calcFootPositionFromSteps();
}

FootStepPlanner::~FootStepPlanner() {}

std::vector<FootPosition> FootStepPlanner::getFootPositions() const {
    return foot_position_;
}

void FootStepPlanner::calcFootPositionFromSteps() {
    foot_position_.clear();
    if (step_length_.empty()) return;
    // Start at origin
    FootPosition current_pos{0.0, 0.0};
    foot_position_.push_back(current_pos);
    for (size_t i = 1; i < step_length_.size(); ++i) {
        current_pos.x += step_length_[i].x;
        current_pos.y = step_length_[i].y;
        foot_position_.push_back(current_pos);
    }
}