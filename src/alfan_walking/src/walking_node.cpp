#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/frames.hpp>
#include <cmath>
#include <memory>
#include <string>

// Panggil header trajectory manager yang sudah kita buat
#include "alfan_walking/foot_trajectory.hpp"

class WalkingNode : public rclcpp::Node {
public:
    WalkingNode() : Node("walking_node"), time_(0.0) {}

    void init(rclcpp::Node::SharedPtr node_ptr) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/legs_controller/commands", 10);

        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");
        while (!parameters_client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Menunggu robot_state_publisher...");
        }
        std::string urdf_xml = parameters_client->get_parameter<std::string>("robot_description");

        this->declare_parameter<std::string>("robot_description", "");
        this->set_parameter(rclcpp::Parameter("robot_description", urdf_xml));

        tracik_r_ = std::make_unique<TRAC_IK::TRAC_IK>(node_ptr, "body", "r_foot", "robot_description", 0.005, 1e-5, TRAC_IK::Speed);
        tracik_l_ = std::make_unique<TRAC_IK::TRAC_IK>(node_ptr, "body", "l_foot", "robot_description", 0.005, 1e-5, TRAC_IK::Speed);

        KDL::Chain chain_r, chain_l;
        tracik_r_->getKDLChain(chain_r);
        tracik_l_->getKDLChain(chain_l);

        KDL::JntArray q_zero_r(chain_r.getNrOfJoints());
        KDL::JntArray q_zero_l(chain_l.getNrOfJoints());
        KDL::SetToZero(q_zero_r);
        KDL::SetToZero(q_zero_l);

        KDL::ChainFkSolverPos_recursive fk_solver_r(chain_r);
        KDL::ChainFkSolverPos_recursive fk_solver_l(chain_l);

        fk_solver_r.JntToCart(q_zero_r, default_pose_r_);
        fk_solver_l.JntToCart(q_zero_l, default_pose_l_);

        q_last_r_ = q_zero_r;
        q_last_l_ = q_zero_l;

        // Injeksi tekukan awal untuk mencegah singularity Jacobian
        if (q_last_r_.data.size() > 3) q_last_r_(3) = 0.1;
        if (q_last_l_.data.size() > 3) q_last_l_(3) = 0.1;

        // // ==========================================================
        // // BLOK KALIBRASI: EKSTRAKTOR YAML INITIAL POSITIONS
        // // ==========================================================
        // double drop_calibration = 0.03; // Kedalaman squat Anda
        
        // KDL::Frame target_squat_r = default_pose_r_;
        // KDL::Frame target_squat_l = default_pose_l_;
        
        // // Terapkan penurunan panggul (lutut menekuk)
        // target_squat_r.p.z(default_pose_r_.p.z() + drop_calibration);
        // target_squat_l.p.z(default_pose_l_.p.z() + drop_calibration);
        
        // KDL::JntArray q_squat_r, q_squat_l;
        // int rc_calib_r = tracik_r_->CartToJnt(q_last_r_, target_squat_r, q_squat_r);
        // int rc_calib_l = tracik_l_->CartToJnt(q_last_l_, target_squat_l, q_squat_l);
        
        // if (rc_calib_r >= 0 && rc_calib_l >= 0) {
        //     std::cout << "\n\n[KALIBRASI SUKSES] Copy-paste teks di bawah ini ke initial_positions.yaml Anda:\n";
        //     std::cout << "--------------------------------------------------------\n";
        //     std::cout << "  # --- Sendi Kaki Kanan ---\n";
        //     std::cout << "  r_hip_yaw: " << q_squat_r(0) << "\n";
        //     std::cout << "  r_hip_roll: " << q_squat_r(1) << "\n";
        //     std::cout << "  r_hip_pitch: " << q_squat_r(2) << "\n";
        //     std::cout << "  r_knee: " << q_squat_r(3) << "\n";
        //     std::cout << "  r_ankle_pitch: " << q_squat_r(4) << "\n";
        //     std::cout << "  r_ankle_roll: " << q_squat_r(5) << "\n\n";
            
        //     std::cout << "  # --- Sendi Kaki Kiri ---\n";
        //     std::cout << "  l_hip_yaw: " << q_squat_l(0) << "\n";
        //     std::cout << "  l_hip_roll: " << q_squat_l(1) << "\n";
        //     std::cout << "  l_hip_pitch: " << q_squat_l(2) << "\n";
        //     std::cout << "  l_knee: " << q_squat_l(3) << "\n";
        //     std::cout << "  l_ankle_pitch: " << q_squat_l(4) << "\n";
        //     std::cout << "  l_ankle_roll: " << q_squat_l(5) << "\n";
        //     std::cout << "--------------------------------------------------------\n\n";
        // } else {
        //     RCLCPP_ERROR(this->get_logger(), "Kalibrasi Gagal! TRAC-IK tidak menemukan solusi squat.");
        // }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), // Loop 100Hz
            std::bind(&WalkingNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // --- 1. PARAMETER TUNING KINEMATIK ---
        double T_cycle = 2.0;               // Durasi 1 siklus penuh (2 langkah)
        double T_half  = T_cycle / 2.0;     // Durasi 1 langkah (1 detik)
        
        double stride  = 0.04;              // Panjang langkah maju (4 cm)
        double lift    = 0.03;              // Tinggi angkat kaki (4 cm)
        double drop    = 0.03;             // Kedalaman squat dasar
        
        double sway_amp = 0.0444;            // Amplitudo goyangan lateral (Y)

        // --- 2. GLOBAL PHASE CLOCK ---
        double phase_time = std::fmod(time_, T_cycle); 

        // --- 3. GENERATOR SWAY CONTINUOUS (Sumbu Y) ---
        // Sinus penuh: Memindahkan beban ke kiri dan kanan dengan mulus di luar Spline.
        // Kita menggunakan phase_time agar sinkron dengan siklus Spline.
        double foot_y_offset = -sway_amp * std::sin((phase_time / T_cycle) * 2.0 * M_PI);

        // --- 4. TRAJECTORY SPLINE MANAGER (Sumbu X dan Z) ---
        if (phase_time < T_half) {
            // FASE A: Kanan Ayun (Swing), Kiri Tumpu (Stance)
            kaki_kanan_.setSwingTrajectory(0.0, T_half,   -stride/2.0, stride/2.0,  0.0, 0.0,  0.0, lift);
            kaki_kiri_.setStanceTrajectory(0.0, T_half,    stride/2.0, -stride/2.0, 0.0, 0.0,  0.0);
            
            target_r_.p = kaki_kanan_.getPosition(phase_time);
            target_l_.p = kaki_kiri_.getPosition(phase_time);
        } else {
            // FASE B: Kiri Ayun (Swing), Kanan Tumpu (Stance)
            double local_time = phase_time - T_half;

            kaki_kiri_.setSwingTrajectory(0.0, T_half,    -stride/2.0, stride/2.0,  0.0, 0.0,  0.0, lift);
            kaki_kanan_.setStanceTrajectory(0.0, T_half,   stride/2.0, -stride/2.0, 0.0, 0.0,  0.0);

            target_r_.p = kaki_kanan_.getPosition(local_time);
            target_l_.p = kaki_kiri_.getPosition(local_time);
        }

        // --- 5. MERGE TRAJECTORY DENGAN DEFAULT POSE URDF ---
        KDL::Frame final_target_r = default_pose_r_;
        KDL::Frame final_target_l = default_pose_l_;

        // Injeksi X (Dari Spline)
        final_target_r.p.x(default_pose_r_.p.x() + target_r_.p.x());
        final_target_l.p.x(default_pose_l_.p.x() + target_l_.p.x());

        // Injeksi Y (Dari Osilator Sinus)
        final_target_r.p.y(default_pose_r_.p.y() + foot_y_offset);
        final_target_l.p.y(default_pose_l_.p.y() + foot_y_offset);

        // Injeksi Z (Dari Spline + Baseline Drop)
        final_target_r.p.z(default_pose_r_.p.z() + target_r_.p.z() + drop);
        final_target_l.p.z(default_pose_l_.p.z() + target_l_.p.z() + drop);

        // --- 6. EKSEKUSI INVERSE KINEMATICS ---
        KDL::JntArray q_out_r, q_out_l;
        int rc_r = tracik_r_->CartToJnt(q_last_r_, final_target_r, q_out_r);
        int rc_l = tracik_l_->CartToJnt(q_last_l_, final_target_l, q_out_l);

        if (rc_r >= 0 && rc_l >= 0) {
            q_last_r_ = q_out_r;
            q_last_l_ = q_out_l;

            auto msg = std_msgs::msg::Float64MultiArray();
            for(uint i = 0; i < q_out_r.data.size(); i++) msg.data.push_back(q_out_r(i));
            for(uint i = 0; i < q_out_l.data.size(); i++) msg.data.push_back(q_out_l(i));
            publisher_->publish(msg);
        } else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "IK Error! Limit sendi atau ukuran tubuh terlampaui. R:%d L:%d", rc_r, rc_l);
        }
        
        time_ += 0.01; // Tambah waktu 10ms
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_r_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_l_;
    
    KDL::JntArray q_last_r_;
    KDL::JntArray q_last_l_;
    KDL::Frame default_pose_r_;
    KDL::Frame default_pose_l_;
    
    // Objek Trajectory Spline untuk masing-masing kaki
    FootTrajectory3D kaki_kanan_;
    FootTrajectory3D kaki_kiri_;
    
    // Variabel penampung hasil hitungan spline (agar tidak bercampur dengan default_pose)
    KDL::Frame target_r_;
    KDL::Frame target_l_;

    double time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WalkingNode>();
    node->init(node); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}