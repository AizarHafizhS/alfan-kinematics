#ifndef FOOT_STEP_PLANNER_HPP
#define FOOT_STEP_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include <vector>

struct FootPosition { // in global coordinate
    double x;  // Foot position in x axis
    double y;  // Foot position in y axis

    // Use if need more advanced walk gait
    // double z;
    // double theta;
};

struct StepLength {  // in local (Body) coordinate
    double x;  // Each step length in x axis
    double y;  // Each step length in y axis

    // Use if need further walk gait
    // double z;        // For inclined walking
    // double theta;    // For changing walk direction
};

class FootStepPlanner {
    public:
        // Constructor
        FootStepPlanner();

        // Destructor
        ~FootStepPlanner();

        // Get planned footsteps
        std::vector<FootPosition> getFootPositions() const;
    private:
        std::vector<StepLength> step_length_;       // Step length array
        std::vector<FootPosition> foot_position_;   // Foot position array

        // Parameter node and client (currently unused)
        rclcpp::Node::SharedPtr parameter_node_;
        std::shared_ptr<rclcpp::SyncParametersClient> parameter_client_;
    
        // Calculate foot positions in global coordinates from step lengths
        void calcFootPositionFromSteps();
};

#endif // FOOT_STEP_PLANNER_HPP