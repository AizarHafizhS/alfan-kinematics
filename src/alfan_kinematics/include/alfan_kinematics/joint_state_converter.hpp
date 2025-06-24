#ifndef JOINT_STATE_CONVERTER_HPP
#define JOINT_STATE_CONVERTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "alfan_kinematics/gait_controller.hpp"
#include "alfan_kinematics/alfan_kinematics.hpp"
#include <utility>

class JointStateConverter {
    public:
        JointStateConverter();
        ~JointStateConverter();

        std::vector<Eigen::VectorXd> convertGait2JointState(
            const WalkControl& controlled_gait, 
            const std::vector<FootPosition>& foot_position, 
            const int& step_idx,
            const int& control_step,
            const double& t);

    private:
        // Constants
        double STEP_DURATION_;      // Duration for one single step
        double DSP_DURATION_;       // Duration of double support phase
        double SSP_DURATION_;       // Duration of single support phase
        double CONTROL_TIMESTEP_;   // Constants to control timing
        double WAIST_HEIGHT_;       // CoM height in z axis (Assume CoM is at waist point)
        double HEIGHT_LEG_LIFT_;    // Height of leg lift during swing
        double WAIST_WIDTH_;        // Width of the waist for foot position calculation

        // Phase definition
        enum class GaitPhase {START, END, FIRST_STEP, LAST_STEP, REGULAR};

        // Parameter node and client (currently unused)
        rclcpp::Node::SharedPtr parameter_node_;
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;

        // Helper functions
        GaitPhase decideGaitPhase(const std::vector<FootPosition>& foot_position, int step);
        std::pair<Eigen::Vector3d, Eigen::Vector3d> calculateFootPosition(
            GaitPhase phase,
            const WalkControl& controlled_gait,
            const std::vector<FootPosition>& foot_position,
            int step_idx,
            int control_step,
            double t
        );
        double calculateSwingTrajectoryPosition(double t);
        double calculateSwingTrajectoryVelocity(double t);
        std::pair<Eigen::VectorXd, Eigen::VectorXd> calculateFootVelocity(const WalkControl& controlled_gait, int control_step, double t);



};

#endif // JOINT_STATE_CONVERTER_HPP