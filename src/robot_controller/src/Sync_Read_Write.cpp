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

    void setupServos(int32_t velocity, int32_t acceleration) {
        const char* log;
        uint16_t model_number;

        for (uint8_t id : dxl_ids) {
            if (!dxl_wb.ping(id, &model_number, &log)) {
                RCLCPP_ERROR(this->get_logger(), "FAILED to ping ID %d: %s", id, log);
            } else {
                RCLCPP_INFO(this->get_logger(), "SUCCEED to ping ID: %d, Model Number: %d", id, model_number);
            }

            if (!dxl_wb.jointMode(id, velocity, acceleration, &log)) {
                RCLCPP_ERROR(this->get_logger(), "FAILED to change joint mode for ID %d: %s", id, log);
            } else {
                RCLCPP_INFO(this->get_logger(), "SUCCEED to change joint mode for ID %d", id);
            }
        }
    }

    void setupSyncHandlers() {
        const char* log;

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

    void moveJoints(std::vector<int32_t>& goal_position) {
        const char* log;
        const uint8_t handler_index = 0;

        if (!dxl_wb.syncWrite(handler_index, goal_position.data(), &log)) {
            RCLCPP_ERROR(this->get_logger(), "FAILED to sync write position: %s", log);
        } else {
            RCLCPP_INFO(this->get_logger(), "SUCCEED sync write position!");
        }

        // for (size_t i = 0; i < dxl_ids.size(); i++) {
        //     if (!dxl_wb.goalPosition(dxl_ids[i], goal_position[i], &log)) {
        //         RCLCPP_ERROR(this->get_logger(), "FAILED to write position for ID %d: %s", dxl_ids[i], log);
        //     } else {
        //         RCLCPP_INFO(this->get_logger(), "SUCCEED write position for ID %d", dxl_ids[i]);
        //     }
        // }
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
        double RAD2VALUE = (180 / M_PI) * (4095 / 360);

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

    void moveLeg(std::vector<double> rightLegPosition, std::vector<double> leftLegPosition, std::vector<int32_t>& present_position) {
        std::vector<int32_t> goal_position(dxl_ids.size(), 0);

        present_position = readPresentPosition();
        printPresentPosition(present_position);
 
        Eigen::VectorXd anglesLeft = ks.computeIK(leftLegPosition, Leg::Left);
        Eigen::VectorXd anglesRight = ks.computeIK(rightLegPosition, Leg::Right);

        Eigen::VectorXd angles = Eigen::VectorXd::Zero(dxl_ids.size());
        angles << anglesRight, anglesLeft;

        goal_position = getGoalPosition(angles);
        printGoalPosition(goal_position);

        moveJoints(goal_position);

        present_position = goal_position;
    }

    void printGoalPosition(std::vector<int32_t> goal_position) {
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "[ID %d] Goal: %d", dxl_ids[i], goal_position[i]);
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

    node->setupServos(50, 0);
    getchar();
    node->setupSyncHandlers();
    getchar();

    int countID = 12;
    std::vector<double> leftLegCoordinates(3, 0);
    std::vector<double> rightLegCoordinates(3, 0);
    std::vector<int32_t> present_position(countID, 0);

    RCLCPP_INFO(node->get_logger(), "Press enter: move to DEFAULT POSITION...");
    getchar();
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position);

    RCLCPP_INFO(node->get_logger(), "Press enter: SHIFT LEFT...");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 1.0};
    leftLegCoordinates = {0.0, 0.0, 1.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position);

    RCLCPP_INFO(node->get_logger(), "Press enter: move to DEFAULT POSITION...");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 0.0};
    leftLegCoordinates = {0.0, 0.0, 0.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position);

    RCLCPP_INFO(node->get_logger(), "Press enter: SHIFT RIGHT...");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 2.0};
    leftLegCoordinates = {0.0, 0.0, 2.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position);

    RCLCPP_INFO(node->get_logger(), "Press enter: move to DEFAULT POSITION...");
    getchar();
    rightLegCoordinates = {0.0, 0.0, 0.0};
    leftLegCoordinates = {0.0, 0.0, 0.0};
    node->moveLeg(rightLegCoordinates, leftLegCoordinates, present_position);

    RCLCPP_INFO(node->get_logger(), "Press enter to END move...");
    getchar();
    node->torqueOff();

    rclcpp::shutdown();
    return 0;
}