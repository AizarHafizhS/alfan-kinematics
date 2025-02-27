#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;
using namespace std;

enum class LegSide { LEFT, RIGHT };

struct Link {
    string name;
    int sister;  // Index of sister joint (-1 if none)
    int child;   // Index of first child joint (-1 if none)
    int mother;  // Index of parent joint
    Vector3d b;  // Fixed offset from parent
    Vector3d a;  // Joint axis (in local coordinates)
    double q;    // Joint angle
    
    // Computed values
    Vector3d p;  // Position in world frame
    Matrix3d R;  // Orientation in world frame
};

class BipedKinematics {
public:
    BipedKinematics() {
        createLegStructure();
        initializeDefaultPose();
    }

    // Main IK interface
    void solveIK(const Vector3d& target_pos, LegSide side) {
        computeFK();  // Update current positions
        
        int foot_id = (side == LegSide::LEFT) ? 7 : 13;
        Link& foot = links[foot_id];
        
        // Calculate IK using Kajita's geometric method
        auto angles = calculateLegIK(foot_id, target_pos);
        
        // Update joint angles
        setJointAngles(angles, side);
        
        // Recompute FK with new angles
        computeFK();
    }

    void printKinematicTree() {
        cout << "Robot Kinematic Tree:\n";
        printLink(1, 0);  // Start from body
    }

private:
    vector<Link> links;
    
    // Robot dimensions (modify according to your robot)
    const Vector3d HIP_OFFSET = {0, 0.044, 0.221};
    const double THIGH_LENGTH = 0.094;
    const double SHIN_LENGTH = 0.093;
    const double ANKLE_HEIGHT = 0.034;

    void createLegStructure() {
        links.resize(15);  // 1 body + 6 joints/leg × 2 legs

        // Body (root link)
        links[1] = {"BODY", -1, 2, -1, 
                   Vector3d::Zero(), Vector3d::Zero(), 0,
                   Vector3d(0, 0, HIP_OFFSET.z()), Matrix3d::Identity()};

        // Left leg
        createLeg(2, LegSide::LEFT);
        // Right leg
        createLeg(8, LegSide::RIGHT);
    }

    void createLeg(int start_idx, LegSide side) {
        double y_sign = (side == LegSide::LEFT) ? 1 : -1;
        
        // Hip Yaw
        links[start_idx] = {
            "HIP_YAW", start_idx + 6, start_idx + 1, 1,
            Vector3d(0, y_sign * HIP_OFFSET.y(), 0),
            Vector3d::UnitZ(), 0, Vector3d::Zero(), Matrix3d::Identity()
        };

        // Hip Roll
        links[start_idx+1] = {
            "HIP_ROLL", -1, start_idx + 2, start_idx,
            Vector3d::Zero(), Vector3d::UnitX(), 0, 
            Vector3d::Zero(), Matrix3d::Identity()
        };

        // Hip Pitch
        links[start_idx+2] = {
            "HIP_PITCH", -1, start_idx + 3, start_idx + 1,
            Vector3d::Zero(), Vector3d::UnitY(), 0,
            Vector3d::Zero(), Matrix3d::Identity()
        };

        // Knee Pitch
        links[start_idx+3] = {
            "KNEE", -1, start_idx + 4, start_idx + 2,
            Vector3d(0, 0, -THIGH_LENGTH), Vector3d::UnitY(), 0,
            Vector3d::Zero(), Matrix3d::Identity()
        };

        // Ankle Pitch
        links[start_idx+4] = {
            "ANKLE_PITCH", -1, start_idx + 5, start_idx + 3,
            Vector3d(0, 0, -SHIN_LENGTH), Vector3d::UnitY(), 0,
            Vector3d::Zero(), Matrix3d::Identity()
        };

        // Ankle Roll (foot)
        links[start_idx+5] = {
            "FOOT", -1, -1, start_idx + 4,
            Vector3d(0, 0, -ANKLE_HEIGHT), Vector3d::UnitX(), 0,
            Vector3d::Zero(), Matrix3d::Identity()
        };
    }

    void initializeDefaultPose() {
        // Set initial joint angles to zero
        for(auto& link : links) {
            link.q = 0;
        }
        computeFK();
    }

    void computeFK(int j = 1) {
        if(j == -1) return;

        if(j != 1) {  // Not body
            int mother = links[j].mother;
            links[j].p = links[mother].R * links[j].b + links[mother].p;
            links[j].R = links[mother].R * rodrigues(links[j].a, links[j].q);
        }

        computeFK(links[j].sister);
        computeFK(links[j].child);
    }

    vector<double> calculateLegIK(int foot_id, const Vector3d& target_pos) {
        vector<double> angles(6);
        const Link& body = links[1];
        const Link& foot = links[foot_id];

        Vector3d r = foot.R.transpose() * (body.p - foot.p);
        double C = r.norm();

        // Knee pitch using cosine law
        double c5 = (C*C - THIGH_LENGTH*THIGH_LENGTH - SHIN_LENGTH*SHIN_LENGTH) 
                   / (2*THIGH_LENGTH*SHIN_LENGTH);
        c5 = clamp(c5, -1.0, 1.0);
        angles[3] = acos(c5);

        // Ankle pitch and roll
        double alpha = asin(THIGH_LENGTH * sin(M_PI - angles[3]) / C);
        angles[4] = -atan2(r.x(), sqrt(r.y()*r.y() + r.z()*r.z())) - alpha;
        angles[5] = atan2(r.y(), r.z());

        // Hip orientation
        Matrix3d R = body.R.transpose() * foot.R 
                   * rodrigues(Vector3d::UnitX(), -angles[5])
                   * rodrigues(Vector3d::UnitY(), -angles[4] - angles[3]);

        // ZXY Euler angles extraction
        angles[0] = atan2(R(1,0), R(0,0));  // Hip yaw
        angles[1] = asin(-R(2,0));          // Hip roll
        angles[2] = atan2(R(2,1), R(2,2));  // Hip pitch

        return angles;
    }

    Matrix3d rodrigues(const Vector3d& axis, double theta) {
        Matrix3d skew;
        skew << 0, -axis.z(), axis.y(),
               axis.z(), 0, -axis.x(),
              -axis.y(), axis.x(), 0;
        
        return Matrix3d::Identity() + 
               sin(theta) * skew + 
               (1 - cos(theta)) * skew * skew;
    }

    void setJointAngles(const vector<double>& angles, LegSide side) {
        int start = (side == LegSide::LEFT) ? 2 : 8;
        for(int i = 0; i < 6; ++i) {
            links[start + i].q = angles[i];
        }
    }

    void printLink(int id, int depth) {
        if(id == -1) return;
        cout << string(depth*2, ' ') 
             << links[id].name << " @ " 
             << links[id].p.transpose() << endl;
        printLink(links[id].child, depth+1);
        printLink(links[id].sister, depth);
    }
};

// Example usage
int main() {
    BipedKinematics biped;
    
    cout << "Initial positions:\n";
    biped.printKinematicTree();

    // Move left foot 10cm forward
    Vector3d target(0.1, 0.044, 0.0);
    biped.solveIK(target, LegSide::LEFT);
    
    cout << "\nAfter IK:\n";
    biped.printKinematicTree();

    return 0;
}