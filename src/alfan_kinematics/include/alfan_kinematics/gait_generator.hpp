#ifndef GAIT_GENERATOR_HPP
#define GAIT_GENERATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "alfan_kinematics/foot_step_planner.hpp"
#include <eigen3/Eigen/Dense>
#include <vector>

struct WalkParameter {
    // All of this using Vector2d because we assume that CoM in z position is constant. 
    // Also, for simpler calculation.
    std::vector<Eigen::Vector2d> com_position;   // Center of mass (COM) position
    std::vector<Eigen::Vector2d> com_velocity;   // Center of mass (COM) velocity
    std::vector<Eigen::Vector2d> next_foot_position; // Next planned foot position based on CoM position
};

class GaitGenerator {
    public:
        // Constructor
        GaitGenerator();

        // Destructor
        ~GaitGenerator();

        // Generate walking gait using LIPM method
        WalkParameter generateGait(const std::vector<FootPosition>& foot_position);
    private:
        // Constants
        double STEP_DURATION_;      // Duration for one single step
        double CONTROL_TIMESTEP_;   // Constants to control timing
        double WAIST_HEIGHT_;       // CoM height in z axis (Assume CoM is at waist point)

        // Parameter node and client (currently unused)
        rclcpp::Node::SharedPtr parameter_node_;
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;

        // Helper functions

        // Calculate modified foot position based on CoM position and velocity
        Eigen::Vector2d calculateModifiedFootPosition(
            const WalkParameter& gait_plan, 
            const std::vector<FootPosition>& foot_position, 
            const Eigen::Vector2d& com_init_pos, 
            const Eigen::Vector2d& com_init_vel, 
            int step, 
            double pos_weight, 
            double vel_weight
        );

        // Update CoM initial state based on gait plan
        void updateComInitialState(
            const WalkParameter& gait_plan, 
            Eigen::Vector2d& current_com_init_pos, 
            Eigen::Vector2d& current_com_init_vel
        );
};

#endif // GAIT_GENERATOR_HPP