#include "rclcpp/rclcpp.hpp"
#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "kinematics/kinematics.hpp"
#include <vector>
#include <string>

class RobotController : public rclcpp::Node {
private:
    DynamixelWorkbench dxl_wb;
    std::string port_name;
    int baud_rate;
    std::vector<uint8_t> dxl_ids;
    std::vector<int64_t> default_position;
    KinematicsSolver ks;

public:
    RobotController() : Node("robot_controller") {
        // Declare and get parameters
        this->declare_parameter<std::string>("port_name", "/dev/ttyUSB0");
        this->declare_parameter<long int>("baud_rate", 1000000);
        this->declare_parameter<std::vector<uint8_t>>("dxl_ids", {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12});
        this->declare_parameter<std::vector<int64_t>>("default_position", {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 1980, 2048});

        this->get_parameter("port_name", port_name);
        this->get_parameter("baud_rate", baud_rate);
        this->get_parameter("dxl_ids", dxl_ids);
        this->get_parameter("default_position", default_position);

        RCLCPP_INFO(this->get_logger(), "RobotController initialized with \n\t-Port: %s, \n\t-Baud rate: %d", port_name.c_str(), baud_rate);
    }

    bool initialize() {
        const char* log;
        if (!dxl_wb.init(port_name.c_str(), baud_rate, &log)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize: %s", log);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "Successfully initialized");
        return true;
    }

    void setupServos() {
        const char* log;
        uint16_t model_number;

        for (uint8_t id : dxl_ids) {
            if (!dxl_wb.ping(id, &model_number, &log)) {
                RCLCPP_ERROR(this->get_logger(), "FAILED to ping ID %d: %s", id, log);
            } else {
                RCLCPP_INFO(this->get_logger(), "SUCCEED to ping ID: %d, Model Number: %d", id, model_number);
            }

            if (!dxl_wb.torqueOn(id, &log)) {
                RCLCPP_ERROR(this->get_logger(), "FAILED to turn on torque for ID %d: %s", id, log);
            } else {
                RCLCPP_INFO(this->get_logger(), "SUCCEED to turn on torque for ID %d", id);
            }
        }
    }

    void setupSyncHandlers() {
        const char* log;

        if (!dxl_wb.addSyncWriteHandler(dxl_ids[0], "Profile_Velocity", &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to add sync write handler: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "Added sync write handler");
        }

        if (!dxl_wb.addSyncWriteHandler(dxl_ids[0], "Goal_Position", &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to add sync write handler: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "Added sync write handler");
        }

        if (!dxl_wb.addSyncReadHandler(dxl_ids[0], "Present_Position", &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to add sync read handler: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "Added sync read handler");
        }
    }

    void torqueOff() {
        const char* log;

        for (uint8_t id : dxl_ids) {
            if (!dxl_wb.torqueOff(id, &log)) {
                RCLCPP_ERROR(this->get_logger(), "FAILED to turn off torque for ID %d: %s", id, log);
            } else {
                RCLCPP_INFO(this->get_logger(), "Torque turned off for ID %d", id);
            }
        }
    }

    void moveJoints(std::vector<int32_t>& goal_position, std::vector<int32_t>& moving_speed) {
        const char* log;
        const uint8_t handler_profile_velocity = 0;
        const uint8_t handler_goal_position = 1;
        
        // Sync write speed/profile velocity
        if (!dxl_wb.syncWrite(handler_profile_velocity, dxl_ids.data(), dxl_ids.size(), moving_speed.data(), 1, &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to sync write velocity: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "SUCCEED sync write velocity!");
        }

        // Sync write position
        if (!dxl_wb.syncWrite(handler_goal_position, dxl_ids.data(), dxl_ids.size(), goal_position.data(), 1, &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to sync write position: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "SUCCEED sync write position!");
        }
    }

    std::vector<int32_t> readPresentPosition() {
        const char* log;
        const uint8_t handler_index = 0;
        std::vector<int32_t> present_position(dxl_ids.size(), 0);

        if (!dxl_wb.syncRead(handler_index, &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to sync read position: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "SUCCEED to sync read position");
        }

        if (!dxl_wb.getSyncReadData(handler_index, present_position.data(), &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to get sync read data: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "SUCCEED to get sync read data");
        }

        return present_position;
    }

    std::vector<int32_t> getGoalPosition(Eigen::VectorXd joint_angles) {
        std::vector<int32_t> goal_position(dxl_ids.size(), 0);
        double RAD2VALUE = (180.0 / M_PI) * (4095.0 / 360.0);

        for (size_t i = 0; i < dxl_ids.size(); i++) {
            bool isReversedID = (dxl_ids[i] == 1) || (dxl_ids[i] == 2) || (dxl_ids[i] == 5) || (dxl_ids[i] == 7) || (dxl_ids[i] == 8) || (dxl_ids[i] == 9) || (dxl_ids[i] == 10);
            int32_t goalValue = joint_angles(i) * RAD2VALUE;

            if (isReversedID) {
                goal_position[i] = default_position[i] - goalValue;
            } else {
                goal_position[i] = default_position[i] + goalValue;
            }
        }

        return goal_position;
    }

    std::vector<int32_t> getMovingSpeed(std::vector<int32_t>& goal_position, std::vector<int32_t>& present_position, double movingTime, int32_t baseSpeed) {
        std::vector<int32_t> positionDifference(dxl_ids.size(), 0);  // Position abs difference between present and goal
        std::vector<int32_t> movingSpeed(dxl_ids.size(), 0);         // Moving speed for each servo in value

        double VAL2RADIAN = (360.0/4095.0) * (M_PI/180.0);    // Converter value to radian
        double valueRPM = 0.229;                              // 0.229 RPM = 1 value (dari spesifikasi MX28)

        for (size_t i = 0; i < dxl_ids.size(); i++){
            // calc difference absolute
            positionDifference[i] = abs(goal_position[i] - present_position[i]);

            // calculate moving speed (from position to value)
            movingSpeed[i] = baseSpeed + (positionDifference[i] * (VAL2RADIAN) *  (1/movingTime) * (60.0 /2*M_PI)  * (1/valueRPM) );
            // VAL2RADIAN untuk ubah posisi ke radian; kemudian ubah ke rad/s dengan (1/movingTime), kemudian rad/s ke RPM, kemudian ke value speed.
            
        }

        return movingSpeed;
    }

    void moveLeg(std::vector<double> rightLegPosition, std::vector<double> leftLegPosition, std::vector<int32_t>& present_position, double movingTime, int32_t base_speed = 0) {
        std::vector<int32_t> goal_position(dxl_ids.size(), 0);
        std::vector<int32_t> moving_speed(dxl_ids.size(), 0);

        present_position = readPresentPosition();
        printPresentPosition(present_position);
 
        Eigen::VectorXd anglesLeft = ks.computeIK(leftLegPosition, Leg::Left);
        Eigen::VectorXd anglesRight = ks.computeIK(rightLegPosition, Leg::Right);

        Eigen::VectorXd angles = Eigen::VectorXd::Zero(dxl_ids.size());
        angles << anglesRight, anglesLeft;

        goal_position = getGoalPosition(angles);
        printGoalPosition(goal_position);
        
        moving_speed = getMovingSpeed(goal_position, present_position, movingTime, base_speed);
        printProfileVelocity(moving_speed);

        moveJoints(goal_position, moving_speed);

        present_position = goal_position;
    }

    void printGoalPosition(std::vector<int32_t> goal_position) {
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "[ID %d] Goal: %d", dxl_ids[i], goal_position[i]);
        }
    }

    void printProfileVelocity(std::vector<int32_t> moving_speed) {
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "[ID %d] Velocity: %d", dxl_ids[i], moving_speed[i]);
        }
    }

    void printPresentPosition(std::vector<int32_t> present_position) {
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "[ID %d] Present: %d", dxl_ids[i], present_position[i]);
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotController>();

    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize RobotController");
        return 1;
    }

    node->setupServos();
    getchar();
    node->setupSyncHandlers();
    getchar();

    // initialize variables
    int countID = 12;
    std::vector<double> leftLegCoordinates(3, 0);
    std::vector<double> rightLegCoordinates(3, 0);
    std::vector<int32_t> present_position(countID, 0);
    std::vector<int32_t> goal_position(countID, 0);
    double move_time = 0;
    double base_speed = 0;

    // set time and base speed (if needed)
    move_time = 5;
    base_speed = 0;

    // 1. DEFAULT
    RCLCPP_INFO(node->get_logger(), "\nPress enter: move to DEFAULT POSITION...");
    getchar();
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position, move_time, base_speed);

    // 2. 
    RCLCPP_INFO(node->get_logger(), "\n!!! NEXT MOVE !!!");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 2.0};
    leftLegCoordinates = {0.0, 0.0, 2.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position, move_time, base_speed);

    // 3.
    RCLCPP_INFO(node->get_logger(), "Press enter: move to DEFAULT POSITION...");
    getchar();
    rightLegCoordinates = {0.0, 3.0, 2.0};
    leftLegCoordinates = {0.0, 3.0, 2.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position, move_time, base_speed);

    // 4.
    RCLCPP_INFO(node->get_logger(), "!!! NEXT MOVE !!!");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 2.0};
    leftLegCoordinates = {0.0, 0.0, 2.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position, move_time, base_speed);

    // 5.
    RCLCPP_INFO(node->get_logger(), "Press enter: move to DEFAULT POSITION...");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 0.0};
    leftLegCoordinates = {0.0, 0.0, 0.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position, move_time, base_speed);

    // 6.
    RCLCPP_INFO(node->get_logger(), "Press enter to END move...");
    getchar();
    node->torqueOff();

    rclcpp::shutdown();
    return 0;
}