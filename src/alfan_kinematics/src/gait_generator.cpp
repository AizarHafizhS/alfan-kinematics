#include "alfan_kinematics/gait_generator.hpp"

GaitGenerator::GaitGenerator() {
    // Make a parameter client for accessing parameters.
    parameter_node_ = rclcpp::Node::make_shared("GaitGeneratorNode");
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(parameter_node_, "ParameterServerNode");
    RCLCPP_INFO(parameter_node_->get_logger(), "Start GAIT GENERATOR.");

    // TODO: Get constants from parameter server

    // For now, manual initialization
    STEP_DURATION_ = 0.8;
    CONTROL_TIMESTEP_ = 0.01;
    WAIST_HEIGHT_ = 0.186;

}

GaitGenerator::~GaitGenerator() {}

// TODO: Make several helper function to make this modular and easy to understand.
WalkParameter GaitGenerator::generateGait(const std::vector<FootPosition>& foot_position) {
    
    WalkParameter planned_walk;  // Output.
    Eigen::Vector2d com_init_pos = {0,0}; // Initial CoM state (x, y)
    Eigen::Vector2d com_init_vel = {0,0}; // Initial CoM state (dx, dy)

    // Indexing and counting
    double t = 0.0; // Time elapsed inside 1 step of walk.
    int step = 0;
    double full_walk_elapsed_time = 0;
    double full_walk_duration = STEP_DURATION_ * (foot_position.size() - 1);

    // Position and velocity weights for optimization
    double pos_weight = 10.0; // Weight for position optimization
    double vel_weight = 1.0;  // Weight for velocity optimization

    // Calculate modified next foot position for optimized result
    planned_walk.next_foot_position.push_back(
        calculateModifiedFootPosition(planned_walk, foot_position, com_init_pos, com_init_vel, step, pos_weight, vel_weight)
    );

    // Calculating position for full walk movement (not just one step)
    while (full_walk_elapsed_time <= full_walk_duration) {
        // Constants
        const double g = 9.81;  // Gravity (m/s^2)
        double Tc = std::sqrt(WAIST_HEIGHT_/g); // Time constant

        // Calculate Sinh and Cosh for every t.
        double S = std::sinh(t/Tc);
        double C = std::cosh(t/Tc);

        // Calculate next CoM position and velocity
        planned_walk.com_position.push_back({
            (com_init_pos.x() - planned_walk.next_foot_position[step].x()) * C + Tc * com_init_vel.x() * S + planned_walk.next_foot_position[step].x(),
            (com_init_pos.y() - planned_walk.next_foot_position[step].y()) * C + Tc * com_init_vel.y() * S + planned_walk.next_foot_position[step].y()
        });
        planned_walk.com_velocity.push_back({
            ((com_init_pos.x() - planned_walk.next_foot_position[step].x()) / Tc) * S + com_init_vel.x() * C,
            ((com_init_pos.y() - planned_walk.next_foot_position[step].y()) / Tc) * S + com_init_vel.y() * C
        });

        if (t >= STEP_DURATION_) {  // Completed 1 step
            step += 1;  // Update step index
            t = 0.0;  // Reset timer

            updateComInitialState(planned_walk, com_init_pos, com_init_vel);

            // Update next modified landing position
            planned_walk.next_foot_position.push_back(
                calculateModifiedFootPosition(planned_walk, foot_position, com_init_pos, com_init_vel, step, pos_weight, vel_weight)
            );

        } else {  // Still stepping
            // Update step timer
            t += CONTROL_TIMESTEP_;
        }

        // Update total time elapsed
        full_walk_elapsed_time += CONTROL_TIMESTEP_;
    }
    
    // Returns output
    return planned_walk;
}

Eigen::Vector2d GaitGenerator::calculateModifiedFootPosition(
    const WalkParameter& gait_plan,
    const std::vector<FootPosition>& foot_position, 
    const Eigen::Vector2d& current_com_init_pos,
    const Eigen::Vector2d& current_com_init_vel,
    int step, 
    double pos_weight, 
    double vel_weight) 
{
    // ===== Declare variables ======
    // This is going to be the output
    Eigen::Vector2d modified_foot_position;

    // Initial CoM state
    double x_init = current_com_init_pos.x();
    double y_init = current_com_init_pos.y();
    double dx_init = current_com_init_vel.x();
    double dy_init = current_com_init_vel.y();

    // Walk primitives
    double x_bar = 0.0;
    double y_bar = 0.0;
    double dx_bar = 0.0;
    double dy_bar = 0.0;

    // Target/Ideal CoM position and velocity
    double x_d = 0.0;   // Target CoM position in x axis
    double y_d = 0.0;   // Target CoM position in y axis
    double dx_d = 0.0;  // Target CoM velocity in x axis
    double dy_d = 0.0;  // Target CoM velocity in y axis

    // Timer and constants
    const double g = 9.81;                  // Gravity (m/s^2)
    double Tc = std::sqrt(WAIST_HEIGHT_/g); // Time constant

    // Sinh and Cosh
    double S = std::sinh(STEP_DURATION_/Tc);
    double C = std::cosh(STEP_DURATION_/Tc);

    // Denominator for the optimizer
    double D = pos_weight * std::pow((C - 1), 2) + vel_weight * std::pow((S / Tc), 2);
    
    // ====== Calculate next foot position ======
    // Update next walk primitives
    x_bar = 0.5 * (foot_position[step + 1].x - foot_position[step].x);
    y_bar = 0.5 * (foot_position[step + 1].y - foot_position[step].y);
    dx_bar = ((C + 1) / (Tc * S)) * x_bar;
    dy_bar = ((C - 1) / (Tc * S)) * y_bar;
    
    // Update next walk target state
    x_d = foot_position[step].x + x_bar;
    y_d = foot_position[step].y + y_bar;
    dx_d = dx_bar;
    dy_d = dy_bar;

    // Modified position x
    modified_foot_position.x() = -1 * ((pos_weight * (C-1)) / D) * (x_d - C * x_init - Tc * S * dx_init) - ((vel_weight* S) / (Tc*D)) * (dx_d - (S/Tc) * x_init - C * dx_init);
    
    // Modified position y
    modified_foot_position.y() = -1 * ((pos_weight * (C-1)) / D) * (y_d - C * y_init - Tc * S * dy_init) - ((vel_weight* S) / (Tc*D)) * (dy_d - (S/Tc) * y_init - C * dy_init);

    return modified_foot_position;
}

void GaitGenerator::updateComInitialState(
    const WalkParameter& gait_plan, 
    Eigen::Vector2d& current_com_init_pos, 
    Eigen::Vector2d& current_com_init_vel) 
{
    // Update CoM state based on the next foot position
    current_com_init_pos.x() = gait_plan.com_position.back().x();
    current_com_init_pos.y() = gait_plan.com_position.back().y();
    current_com_init_vel.x() = gait_plan.com_velocity.back().x();
    current_com_init_vel.y() = gait_plan.com_velocity.back().y();
}