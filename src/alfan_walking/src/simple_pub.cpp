#include "alfan_msgs/msg/joint_command.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class JointPublisher : public rclcpp::Node {
	public:
	// Constructor
	JointPublisher() : Node("joint_pub") {
		pub_ = this->create_publisher<alfan_msgs::msg::JointCommand>("DynamixelController/command", 10);
		sub_ = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&JointPublisher::sub_cb, this, _1));
		// timer_ = this->create_wall_timer(2000ms, std::bind(&JointPublisher::timer_cb,s this));
	}

	private:
	// Callback
	void timer_cb() {
		auto msg = alfan_msgs::msg::JointCommand();
		msg.joint_names = {"r_hand_24", "l_hand_34"};
		msg.positions = {-1.57, -1.57};
		msg.velocities = {5.0, 5.0};
		// msg.accelerations = {}
		pub_->publish(msg);
		RCLCPP_INFO(this->get_logger(), "Publish to '%s'", msg.joint_names[0].c_str());
	}

	void sub_cb(const sensor_msgs::msg::JointState msg) const {
		int length = sizeof(msg.position)/sizeof(msg.position[0]);

		for(int i = 0; i < length; i++) {
			if (msg.position[5] <= -1.56) {
				// publish
				auto cmd = alfan_msgs::msg::JointCommand();
				cmd.joint_names = {"r_hand_24", "l_hand_34"};
				cmd.positions = {0.0, 0.0};
				cmd.velocities = {5.0, 5.0};
				// cmd.accelerations = {}
				pub_->publish(cmd);
			} else if (msg.position[5] >= -0.01) {
				auto cmd = alfan_msgs::msg::JointCommand();
				cmd.joint_names = {"r_hand_24", "l_hand_34"};
				cmd.positions = {-1.57, -1.57};
				cmd.velocities = {5.0, 5.0};
				// cmd.accelerations = {}
				pub_->publish(cmd);
			}
		}
	}

	// Publisher
	rclcpp::Publisher<alfan_msgs::msg::JointCommand>::SharedPtr pub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
	rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<JointPublisher>();
	rclcpp::spin(node);

	rclcpp::shutdown();
	return 0;
}
