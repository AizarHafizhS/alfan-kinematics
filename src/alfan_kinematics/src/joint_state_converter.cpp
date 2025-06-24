#include "alfan_kinematics/joint_state_converter.hpp"
#include <iostream>

JointStateConverter::JointStateConverter() {
    // Make a parameter client for accessing parameters.
    parameter_node_ = rclcpp::Node::make_shared("JointStateConverterNode");
    parameter_client_ = std::make_shared<rclcpp::SyncParametersClient>(parameter_node_, "ParameterServerNode");
    RCLCPP_INFO(parameter_node_->get_logger(), "Start JOINT STATE CONVERTER.");

    // TODO: Get constants from parameter server
    // For now, manual initialization
    // Constants for gait control
    STEP_DURATION_     = 0.8;
    DSP_DURATION_      = 0.2;
    SSP_DURATION_      = STEP_DURATION_ - DSP_DURATION_;
    CONTROL_TIMESTEP_  = 0.01;
    WAIST_HEIGHT_      = 0.186;
    HEIGHT_LEG_LIFT_   = 0.025;
    WAIST_WIDTH_       = 0.1;
}

JointStateConverter::~JointStateConverter() {
    // Destructor
}
std::vector<Eigen::VectorXd> JointStateConverter::convertGait2JointState(
            const WalkControl& controlled_gait, 
            const std::vector<FootPosition>& foot_position, 
            const int& step,
            const int& control_step,
            const double& t) 
{
    std::vector<Eigen::VectorXd> joint_states;

    // Decide gait phase
    GaitPhase phase = decideGaitPhase(foot_position, step);

    // Check if step index is valid
    if (phase == GaitPhase::REGULAR && step >= foot_position.size() - 1) {
        std::cerr << "Warning: Step index exceeds foot position size." << std::endl;
        return joint_states; // Return empty vector if step index is out of bounds
    }

    // Calculate foot positions based on gait phase
    auto [support_foot_pos, swing_foot_pos] = calculateFootPosition(
        phase, controlled_gait, foot_position, step, control_step, t
    );
    // Calculate foot velocities
    auto [support_foot_vel, swing_foot_vel] = calculateFootVelocity(controlled_gait, control_step, t);

    // Create joint state vectors
    std::vector<double> left_leg_pos;
    std::vector<double> right_leg_pos;
    std::vector<double> left_leg_vel;
    std::vector<double> right_leg_vel;

    // HElper variables for foot positions

    // Set left and right leg positions based on support and swing foot positions
    if(phase == GaitPhase::START || phase == GaitPhase::END) {
        // For START and END phases, use the next or previous foot position
        FootPosition ref_foot_pos = (phase == GaitPhase::START) ? foot_position[step + 1] : foot_position[step - 1];
        if (ref_foot_pos.y >= 0) { // Left foot support
            left_leg_pos = {support_foot_pos.x(), support_foot_pos.y(), support_foot_pos.z()};
            right_leg_pos = {swing_foot_pos.x(), swing_foot_pos.y(), swing_foot_pos.z()};
            // TODO: Jacobian calculation for velocity
            left_leg_vel = {support_foot_vel.x(), support_foot_vel.y(), support_foot_vel.z()};
            right_leg_vel = {swing_foot_vel.x(), swing_foot_vel.y(), swing_foot_vel.z()};

        } else { // Right foot support
            left_leg_pos = {swing_foot_pos.x(), swing_foot_pos.y(), swing_foot_pos.z()};        
            right_leg_pos = {support_foot_pos.x(), support_foot_pos.y(), support_foot_pos.z()};
            // TODO: Jacobian calculation for velocity
            left_leg_vel = {swing_foot_vel.x(), swing_foot_vel.y(), swing_foot_vel.z()};
            right_leg_vel = {support_foot_vel.x(), support_foot_vel.y(), support_foot_vel.z()};
        }
    } else if (foot_position[step].y > 0) {
        
    }
}

JointStateConverter::GaitPhase JointStateConverter::decideGaitPhase(
    const std::vector<FootPosition>& foot_pos, int step) 
{
    FootPosition curr_foot_pos = foot_pos[step];
    FootPosition next_foot_pos = foot_pos[step + 1];
    FootPosition prev_foot_pos = foot_pos[step - 1];

    int total_steps = (int)foot_pos.size();

    if (step >= total_steps) return GaitPhase::REGULAR;
    
    if (curr_foot_pos.y == 0) {
        return (step == 0) ? GaitPhase::START : GaitPhase::END;
    }
    else if (step > 0 && (prev_foot_pos.y == 0)) {
        return GaitPhase::FIRST_STEP;
    }
    else if ((next_foot_pos.y == 0)) {
        return GaitPhase::LAST_STEP;
    }
    return GaitPhase::REGULAR;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> JointStateConverter::calculateFootPosition(
    GaitPhase phase,
    const WalkControl& controlled_gait,
    const std::vector<FootPosition>& foot_pos,
    int step,
    int control_step,
    double t) 
{
    Eigen::Vector3d support_foot_pos;
    Eigen::Vector3d swing_foot_pos;

    // Com reference
    const double com_x = controlled_gait.com_position_fix[control_step].x();
    const double com_y = controlled_gait.com_position_fix[control_step].y();

    // ZMP reference
    const double curr_zmp_x = controlled_gait.zmp_fix[step].x();
    const double curr_zmp_y = controlled_gait.zmp_fix[step].y();
    const double prev_zmp_x = controlled_gait.zmp_fix[step - 1].x();
    const double prev_zmp_y = controlled_gait.zmp_fix[step - 1].y();
    const double next_zmp_x = controlled_gait.zmp_fix[step + 1].x();
    const double next_zmp_y = controlled_gait.zmp_fix[step + 1].y();

    // Foot position reference
    FootPosition curr_foot_pos = foot_pos[step];
    FootPosition next_foot_pos = foot_pos[step + 1];
    FootPosition prev_foot_pos = foot_pos[step - 1];

    // Timing definition
    const double start_ssp = DSP_DURATION_ / 2; // Start of single support phase
    const double end_ssp = STEP_DURATION_ - (DSP_DURATION_ / 2); // End of single support phase

    // Calculate swing trajectory point and velocity
    double swing_trajectory = calculateSwingTrajectoryPosition(t);

    switch (phase)  {
        case GaitPhase::START:
        case GaitPhase::END: {
            const bool is_start = (phase == GaitPhase::START);
            FootPosition ref_foot_pos = is_start ? next_foot_pos : prev_foot_pos;
    
            if (ref_foot_pos.y >= 0) { // Left foot support
                support_foot_pos = {
                    curr_zmp_x - com_x,
                    WAIST_WIDTH_ - com_y,
                    0
                };
                swing_foot_pos = {
                    curr_zmp_x - com_x,
                    -WAIST_WIDTH_ - com_y,
                    0
                };
            } else { // Right foot support
                support_foot_pos = {
                    curr_zmp_x - com_x,
                    -WAIST_WIDTH_ - com_y,
                    0
                };
                swing_foot_pos = {
                    curr_zmp_x - com_x,
                    WAIST_WIDTH_ - com_y,
                    0
                };
            }
            break;
        }
    
        case GaitPhase::FIRST_STEP: {
            support_foot_pos = {
                curr_zmp_x - com_x,
                curr_zmp_y - com_y,
                0
            };

            if (t <= start_ssp) {  // In first half of double support phase
                swing_foot_pos = {
                    prev_zmp_x - com_x,
                    next_zmp_y - com_y,  // CHECKME: Maybe need interpolation?
                    0
                };
            } else if (t >= end_ssp) {  // In second half of double support phase
                swing_foot_pos = {
                    next_zmp_x - com_x,
                    next_zmp_y - com_y,
                    0
                };
            } else {  // In single support phase
                swing_foot_pos = {
                    (next_zmp_x - prev_zmp_x) * (t - start_ssp) / SSP_DURATION_,
                    next_zmp_y - com_y,
                    swing_trajectory
                };
            }
            break;
        }

        case GaitPhase::LAST_STEP: {
            support_foot_pos = {
                curr_zmp_x - com_x,
                curr_zmp_y - com_y,
                0
            };

            if (t <= start_ssp) {  // In first half of double support phase
                swing_foot_pos = {
                    prev_zmp_x - com_x,
                    prev_zmp_y - com_y,
                    0
                };
            } else if (t >= end_ssp) {  // In second half of double support phase
                swing_foot_pos = {
                    next_zmp_x - com_x,
                    prev_zmp_y - com_y,
                    0
                };
            } else {  // In single support phase
                swing_foot_pos = {
                    (next_zmp_x - prev_zmp_x) * (t - start_ssp) / SSP_DURATION_ - (next_zmp_x - prev_zmp_x),
                    prev_zmp_y - com_y,
                    swing_trajectory
                };
            }
            break;
        }
        
        case GaitPhase::REGULAR: {
            support_foot_pos = {
                curr_zmp_x - com_x,
                curr_zmp_y - com_y,
                0
            };

            if (t <= start_ssp) {  // In first half of double support phase
                swing_foot_pos = {
                    prev_zmp_x - com_x,
                    prev_zmp_y + ((next_zmp_y - prev_zmp_y) * t/STEP_DURATION_) - com_y,
                    0
                };
            } else if (t >= end_ssp) {  // In second half of double support phase
                swing_foot_pos = {
                    next_zmp_x - com_x,
                    prev_zmp_y + ((next_zmp_y - prev_zmp_y) * t/STEP_DURATION_) - com_y,
                    0
                };
            } else {  // In single support phase
                swing_foot_pos = {
                    ((next_zmp_x - prev_zmp_x) * (t - start_ssp) / SSP_DURATION_) - (next_zmp_x - prev_zmp_x)/2,
                    prev_zmp_y + ((next_zmp_y - prev_zmp_y) * t/STEP_DURATION_) - com_y,
                    swing_trajectory
                };
            }
            break;
        }
    }
    return {support_foot_pos, swing_foot_pos};
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> JointStateConverter::calculateFootVelocity(
    const WalkControl& controlled_gait, int control_step, double t) 
{
    Eigen::VectorXd support_foot_vel(6);
    Eigen::VectorXd swing_foot_vel(6);

    support_foot_vel = {
        controlled_gait.com_velocity_fix[control_step].x(),
        controlled_gait.com_velocity_fix[control_step].y(),
        0.0,  // z velocity is assumed to be zero
        0.0,  // roll velocity is assumed to be zero
        0.0,  // pitch velocity is assumed to be zero
        0.0   // yaw velocity is assumed to be zero
    };
    swing_foot_vel = {
        controlled_gait.com_velocity_fix[control_step].x(),
        controlled_gait.com_velocity_fix[control_step].y(),
        calculateSwingTrajectoryVelocity(t),  // z velocity based on swing trajectory
        0.0,  // roll velocity is assumed to be zero
        0.0,  // pitch velocity is assumed to be zero
        0.0   // yaw velocity is assumed to be zero
    };
    return {support_foot_vel, swing_foot_vel};
}

double JointStateConverter::calculateSwingTrajectoryPosition(double t) {
    double swing_trajectory = 0.0;  // Swing leg trajectory in z axis

    double start_ssp = DSP_DURATION_ / 2;  // Start of single support phase
    double end_ssp = STEP_DURATION_ - DSP_DURATION_ / 2;  // End of single support phase
    if (t < start_ssp || t > end_ssp) {
        swing_trajectory = 0.0;  // Not in single support phase
    }

    double omega = M_PI / SSP_DURATION_;  // Angular frequency
    double t_aksen = t - start_ssp;  // Time since the start of single support phase

    swing_trajectory = HEIGHT_LEG_LIFT_ * std::sin(omega * t_aksen);  // Swing trajectory in z axis

    return swing_trajectory;
}

double JointStateConverter::calculateSwingTrajectoryVelocity(double t) {
    double swing_trajectory_velocity = 0.0;  // Swing leg trajectory velocity in z axis

    double start_ssp = DSP_DURATION_ / 2;  // Start of single support phase
    double end_ssp = STEP_DURATION_ - DSP_DURATION_ / 2;  // End of single support phase
    if (t < start_ssp || t > end_ssp) {
        swing_trajectory_velocity = 0.0;  // Not in single support phase
    }

    double omega = M_PI / SSP_DURATION_;  // Angular frequency
    double t_aksen = t - start_ssp;  // Time since the start of single support phase

    swing_trajectory_velocity = HEIGHT_LEG_LIFT_ * omega * std::cos(omega * t_aksen);  // Swing trajectory velocity in z axis

    return swing_trajectory_velocity;
}