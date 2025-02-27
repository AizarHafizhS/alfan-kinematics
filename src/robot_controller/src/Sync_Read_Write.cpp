#include "dynamixel_workbench_toolbox/dynamixel_workbench.h"
#include "kinematics/kinematics.hpp"
#include <iostream>
#include <vector>
#include <unistd.h>

using namespace std;

class RobotController {
private:
    DynamixelWorkbench dxl_wb;                  ///< Dynamixel workbench instance
    const char* port_name = "/dev/ttyUSB0";     ///< Port to U2D2
    int baud_rate = 57600;                      ///< Baudrate for every servo
    vector<uint8_t> dxl_ids = {2, 3, 4, 5, 6};  ///< Servo IDs
    KinematicsSolver ks;                        ///< Kinematics Solver instance

public:
    /**
     * @brief Initialize communication to U2D2
     * @returns Connected or not
     */
    bool initialize() {
        const char* log;    ///< Store log message (NULL or !NULL)
        
        /// If not connected
        if (!dxl_wb.init(port_name, baud_rate, &log)) { 
            printf("FAILED to initialize\n%s\n", log);
            return false;
        }

        /// If connected
        printf("Successfully initialized\n");
        return true;
    }

    /**
     * @brief Setting up servo (mode, torque, velocity (optional))
     * @returns 
     */
    void setupServos(int32_t velocity, int32_t acceleration) {
        const char* log;        ///< Store log messages
        uint16_t model_number;  ///< Servo model number

        /// Start setup every servo
        for (uint8_t id : dxl_ids) {
            /// Ping servo
            if (!dxl_wb.ping(id, &model_number, &log)) { /// If failed to Ping
                printf("FAILED to ping ID %d\n%s\n", id, log);
            } else {
                printf("SUCCEED to ping ID: %d, Model Number: %d\n", id, model_number);
            }

            /// Set joint mode 
            if (!dxl_wb.jointMode(id, velocity, acceleration, &log)) { /// If failed to set joint mode
                printf("FAILED to change joint mode for ID %d\n%s\n", id, log);
            } else {
                printf("SUCCEED to change joint mode for ID %d\n", id);
            }
        }
    }

    /**
     * @brief Set instruction (Sync write and sync read) handler
     * @returns (Void)
     */
    void setupSyncHandlers() {
        const char* log;    ///< Store log message

        /// Start add Sync write handlers
        if (!dxl_wb.addSyncWriteHandler(dxl_ids[0], "Goal_Position", &log)) { /// If fail to add handler
            printf("FAILED to add sync write handler\n%s\n", log);
        } else {
            printf("Added sync write handler\n");
        }
        
        /// Start add Sync read handlers
        if (!dxl_wb.addSyncReadHandler(dxl_ids[0], "Present_Position", &log)) { /// If fail to add handler
            printf("FAILED to add sync read handler\n%s\n", log);
        } else {
            printf("Added sync read handler\n");
        }
    }

    /**
     * @brief Turn off torque for all servo (untuk percobaan aja)
     * @returns (void)
     */
    void torqueOff() {
        const char* log;    ///< Store log message

        /// Start turning off torque
        for (uint8_t id : dxl_ids) {
            if (!dxl_wb.torqueOff(id, &log)) { /// If fail to turn off torque
                printf("FAILED to turn off torque for ID %d\n%s\n", id, log);
            } else {
                printf("Torque turned off for ID %d\n", id);
            }
        }
    }

    /**
     * @brief Move joints with Sync Write instruction
     * @param goal_position An array of each corresponding servo's goal position
     * @returns (void)
     */
    void moveJoints(vector<int32_t>& goal_position) {
        const char* log;                                        ///< Store log message
        const uint8_t handler_index = 0;                        ///< Handler index for sync instruction

        /// Start sync write
        if (!dxl_wb.syncWrite(handler_index, goal_position.data(), &log)) { /// If fail to sync write
            printf("FAILED to sync write position\n%s\n", log);
        } else {
            printf("SUCCEED sync write position!\n");
        }
    }

    /**
     * @brief Read all servo present position
     * @returns present_position - An array of every servo's present position
     */
    vector<int32_t> readPresentPosition() {
        const char* log;                                        ///< Store log message
        const uint8_t handler_index = 0;                        ///< Handler index for sync instruction
        vector<int32_t> present_position(dxl_ids.size(), 0);    ///< Present position of each servo

        /// Start read position data
        if (!dxl_wb.syncRead(handler_index, &log)) { /// If fail to sync read
            printf("FAILED to sync read position\n%s\n", log);
        } else {
            printf("SUCCEED to sync read position\n");
        }

        /// Start getting and storing position data read before 
        if (!dxl_wb.getSyncReadData(handler_index, present_position.data(), &log)) { /// If fail to get position data
            printf("FAILED to get sync read data\n%s\n", log);
        } else {
            printf("SUCCEED to get sync read data\n");
        }

        return present_position;
    }

    /**
     * @brief Loop to control all process
     * @returns (void)
     */
    void controlLoop(double x, double y, double z) {
        vector<int32_t> present_position(dxl_ids.size(), 0);    ///< Array of every servo's present position
        vector<int32_t> initial_position(dxl_ids.size(), 2048); ///< Initial position of every servo (set to 2048 (180 derajat))
        Eigen::VectorXd angles;                                 ///< Angle of each joint

        /// Read current position
        present_position = readPresentPosition();

        /// Compute inverse kinematics
        angles = ks.computeIK(x, y, z, Leg::Right);

        ks.computeFK(1);            /// Compute forward kinematics
        ks.printPositions();        /// Print EXPECTED position
        cout << "cont..." << endl;
        getchar();                  /// Wait for ENTER click
        
        vector<int32_t> goal_position(dxl_ids.size(), 0);           ///< Store goal position in an array
        goal_position = getGoalPosition(initial_position, angles);  /// Get new goal position in value (0-4095)
        
        cout << "Press enter to move to INIT POSITION..." << endl;
        getchar();                                  /// Wait for ENTER
        moveJoints(initial_position);               /// Move to initial position
        cout << "continue to read position... \n" << endl;
        getchar();
        present_position = readPresentPosition();   /// Read present position
        printPresentPosition(present_position);     /// Print present position

        cout << "Press enter to move to NEXT GOAL POSITION..." << endl;
        getchar();
        moveJoints(goal_position);                  /// Move to initial position
        cout << "continue to read position... \n" << endl;
        getchar();
        present_position = readPresentPosition();   /// Read present position
        printPresentPosition(present_position);     /// Print present position

        cout << "Press enter to END move..." << endl;
        getchar();
        torqueOff();
    }

    /**
     * @brief Calculate goal position from radian to value (0-4095) based on present position
     * @param present_position Array of present position (from Sync read result or expected present position)
     * @param joint_angles Vector of desired joint angles in radian
     * @returns goal_position - an array of goal position in value range (0-4095) based on present position
     */
    vector<int32_t> getGoalPosition (vector<int32_t> present_position, Eigen::VectorXd joint_angles) {
        vector<int32_t> goal_position = present_position;   ///< Goal position first initialize is same to present position
        double RAD2VALUE = (180/M_PI) * (4095/360);         ///< Converter from radian to value (0-4095)
        int id_count = dxl_ids.size();
        
        /// Start calculating
        for (int i = 0; i < id_count; i++) {
            if ((dxl_ids[i] == 1) || (dxl_ids[i] == 2) || (dxl_ids[i] == 5)) { /// ID 1, 2, 5 arah value dan tanda hasil angle berbeda
                goal_position[i] -= static_cast<int32_t>(joint_angles(i+1) * RAD2VALUE);
            } else {
                goal_position[i] += static_cast<int32_t>(joint_angles(i+1) * RAD2VALUE);
            }
        }
        return goal_position;
    }

    void printGoalPosition(vector<int32_t> goal_position){
        /// Print goal position
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            printf("[ID %d] Goal: %d\n", dxl_ids[i], goal_position[i]);
        }
    }

    void printPresentPosition(vector<int32_t> present_position){
        for (size_t i = 0; i < dxl_ids.size(); i++) {
            printf("[ID %d] Present: %d\n", dxl_ids[i], present_position[i]);
        }
    }
};

int main() {
    RobotController robot;
    if (!robot.initialize()) return 0;
    robot.setupServos(80, 0);
    getchar();
    robot.setupSyncHandlers();
    getchar();
    robot.controlLoop(0, 0, 0.025);
    return 0;
}