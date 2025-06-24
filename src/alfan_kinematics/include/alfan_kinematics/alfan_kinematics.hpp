#ifndef ALFAN_KINEMATICS_HPP
#define ALFAN_KINEMATICS_HPP

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

/**
 * @brief Enum class to specify the leg type.
 */
enum class Leg {
    LEFT,
    RIGHT
};

/**
 * @brief Structure representing a robot link.
 */
struct Link {
    std::string name;         // Name of the link
    int sister;               // Index of the sister link
    int child;                // Index of the child link
    int mother;               // Index of the parent link
    Eigen::Vector3d p;        // Absolute position (in meters)
    Eigen::Matrix3d R;        // Orientation (rotation matrix)
    Eigen::Vector3d b;        // Offset from parent (in meters)
    Eigen::Vector3d a;        // Joint axis
    double q;                 // Joint angle (in radians)
};

/**
 * @brief Kinematics solver for humanoid robot legs.
 *
 * This class provides forward and inverse kinematics routines and is designed to be
 * integrated into higher-level control systems (e.g., trajectory planning, stability control).
 */
class KinematicsSolver {
public:
    /**
     * @brief Constructor.
     */
    KinematicsSolver();

    /**
     * @brief Destructor.
     */
    ~KinematicsSolver();

    /**
     * @brief Recursively computes forward kinematics starting from a given link index.
     * @param j The starting link index.
     */
    void computeFK(int j);

    /**
     * @brief Computes inverse kinematics for the specified leg.
     * @param leg The leg (Left or Right) for which IK is computed.
     * @return A 6-element vector containing the computed joint angles.
     */
    Eigen::VectorXd computeIK(std::vector<double> legPosition, Leg leg);

    /**
     * @brief Recursively traverses and prints the kinematic tree (for DEBUG).
     * @param id The starting link index.
     * @param depth The current depth in the tree (default is 0).
     */
    void traverseTree(int id, int depth = 0);

    
    /**
     * @brief print foot positions ((for DEBUG)).
     */
    void printPositions();
    
    private:
    std::vector<Link> uLINK;  // Vector storing all robot links.
    
    // Robot geometry parameters (in meters) declared as compile-time constants.
    static constexpr double HIP_OFFSET_Y = 0.044;
    static constexpr double HIP_HEIGHT   = 0.1868;   // Dihitung dari ankle, bukan dari telapak kaki.
    static constexpr double KNEE_HEIGHT  = 0.093;
    static constexpr double ANKLE_HEIGHT = 0.034;   // Ankle dianggap ujung kaki.
    static constexpr double THIGH_LENGTH = 0.0938;
    static constexpr double SHIN_LENGTH  = 0.093;
    // static constexpr double FOOT_HEIGHT  = 0.034;
    
    /**
     * @brief Computes a rotation matrix using Rodrigues' formula.
     * @param w The rotation axis.
     * @param theta The rotation angle (in radians).
     * @return The resulting rotation matrix.
     */
    Eigen::Matrix3d rodrigues(const Eigen::Vector3d& w, double theta);
    
    /**
     * @brief Initializes the robot links.
     * @param uLINK A reference to the vector that stores the links.
     */
    void initLinks(std::vector<Link>& uLINK);
    
    /**
     * @brief Adjusts the foot position for the specified leg.
     * @param desiredFootPos The desired change in foot position (in meters).
     * @param leg The leg (Left or Right) to update.
     */
    void setFootPos(const Eigen::Vector3d desiredFootPos, Leg leg);

    /**
     * @brief Sets joint angles for the specified leg.
     * @param jointAngles A vector of 6 joint angles.
     * @param leg The leg (Left or Right) to update.
     */
    void setJointAngles(const Eigen::VectorXd& jointAngles, Leg leg);
};


#endif // ALFAN_KINEMATICS_HPP
