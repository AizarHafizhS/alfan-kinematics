#include "alfan_kinematics/gait_controller.hpp"

GaitController::GaitController() {
    parameter_node_ = rclcpp::Node::make_shared("GaitControllerNode");
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(parameter_node_, "ParameterServerNode");

    RCLCPP_INFO(parameter_node_->get_logger(), "Start GAIT CONTROLLER.");
}

GaitController::~GaitController() {}

WalkControl GaitController::controlGait(const WalkParameter& planned_gait) {
    WalkControl fixed_gait;

    fixed_gait.com_position_fix = planned_gait.com_position;
    fixed_gait.com_velocity_fix = planned_gait.com_velocity;
    fixed_gait.zmp_fix          = planned_gait.next_foot_position;

    return fixed_gait;
}

