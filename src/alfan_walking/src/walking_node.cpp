#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/frames.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <cmath>
#include <memory>
#include <string>

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

        std::string base_link = "body"; 
        std::string r_tip_link = "r_foot";
        std::string l_tip_link = "l_foot";

        double timeout = 0.005; 
        double eps = 1e-5;

        tracik_r_ = std::make_unique<TRAC_IK::TRAC_IK>(node_ptr, base_link, r_tip_link, "robot_description", timeout, eps, TRAC_IK::Speed);
        tracik_l_ = std::make_unique<TRAC_IK::TRAC_IK>(node_ptr, base_link, l_tip_link, "robot_description", timeout, eps, TRAC_IK::Speed);

        KDL::Chain chain_r, chain_l;
        if (!tracik_r_->getKDLChain(chain_r) || !tracik_l_->getKDLChain(chain_l)) {
            RCLCPP_FATAL(this->get_logger(), "Gagal membaca Rantai Kinematika!");
            return;
        }

        // --- FASE DETEKSI GEOMETRI ---
        KDL::JntArray q_zero_r(chain_r.getNrOfJoints());
        KDL::JntArray q_zero_l(chain_l.getNrOfJoints());
        KDL::SetToZero(q_zero_r);
        KDL::SetToZero(q_zero_l);

        KDL::ChainFkSolverPos_recursive fk_solver_r(chain_r);
        KDL::ChainFkSolverPos_recursive fk_solver_l(chain_l);

        fk_solver_r.JntToCart(q_zero_r, default_pose_r_);
        fk_solver_l.JntToCart(q_zero_l, default_pose_l_);

        RCLCPP_INFO(this->get_logger(), "Matriks Aktif! Posisi Lurus (Z): Kanan=%.3f, Kiri=%.3f", default_pose_r_.p.z(), default_pose_l_.p.z());

        q_last_r_ = q_zero_r;
        q_last_l_ = q_zero_l;

        // Beri tebakan tekukan awal 0.1 radian pada lutut agar IK tahu cara menekuk (Mencegah Jacobian 0)
        // Pada robot Anda, index sendi r_knee dan l_knee kemungkinan ada di index ke-3
        if (q_last_r_.data.size() > 3) q_last_r_(3) = 0.1;
        if (q_last_l_.data.size() > 3) q_last_l_(3) = 0.1;

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&WalkingNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        // Karena Z Lurus adalah angka terendahnya (mendekati lantai), 
        // kita tambahkan angka positif ke Z untuk "mengangkat" kaki (menekuk lutut).
        // Fungsi cosinus ini akan menghasilkan jarak angkat (lift) dari 0 cm hingga 4 cm (0.04 m).
        double lift_amount = 0.02 * (1.0 - std::cos(M_PI * time_)); 
        
        KDL::Frame target_r = default_pose_r_;
        KDL::Frame target_l = default_pose_l_;

        // Eksekusi Kompresi Kaki. Rotasi & XY dibiarkan murni sesuai bawaan URDF.
        target_r.p.z(default_pose_r_.p.z() + lift_amount); 
        target_l.p.z(default_pose_l_.p.z() + lift_amount);

        KDL::JntArray q_out_r, q_out_l;

        int rc_r = tracik_r_->CartToJnt(q_last_r_, target_r, q_out_r);
        int rc_l = tracik_l_->CartToJnt(q_last_l_, target_l, q_out_l);

        if (rc_r >= 0 && rc_l >= 0) {
            q_last_r_ = q_out_r;
            q_last_l_ = q_out_l;

            auto msg = std_msgs::msg::Float64MultiArray();
            msg.data.clear();
            
            for(uint i = 0; i < q_out_r.data.size(); i++) { msg.data.push_back(q_out_r(i)); }
            for(uint i = 0; i < q_out_l.data.size(); i++) { msg.data.push_back(q_out_l(i)); }
            
            publisher_->publish(msg);
        } else {
            // Jika error masih muncul, artinya kompresi 4cm (lift_amount) menabrak batas joint_limits di URDF.
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "IK Error! Kanan: %d | Kiri: %d. Tidak dapat menekuk sejauh %.3f meter.", rc_r, rc_l, lift_amount);
        }
        
        time_ += 0.01; 
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_r_;
    std::unique_ptr<TRAC_IK::TRAC_IK> tracik_l_;
    
    KDL::JntArray q_last_r_;
    KDL::JntArray q_last_l_;
    KDL::Frame default_pose_r_;
    KDL::Frame default_pose_l_;
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