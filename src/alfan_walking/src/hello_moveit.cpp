// Program coba coba doang
// Untuk nyoba gerakin kaki di RViz pake code dan API Moveit2.
// Feel free untuk hapus/modify code nya

#include <memory>
#include <thread> // Added for threading

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS
  rclcpp::init(argc, argv);
  
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "left_leg");

  // Get the current pose of the end-effector
  auto current_pose_stamped = move_group_interface.getCurrentPose();
  auto current_pose = current_pose_stamped.pose;

  // Print the current valid pose and the reference frame to the terminal
  RCLCPP_INFO(logger, "--- CURRENT POSE ---");
  RCLCPP_INFO(logger, "Reference Frame: %s", current_pose_stamped.header.frame_id.c_str());
  RCLCPP_INFO(logger, "Position    - x: %.3f, y: %.3f, z: %.3f", 
              current_pose.position.x, current_pose.position.y, current_pose.position.z);
  RCLCPP_INFO(logger, "Orientation - x: %.3f, y: %.3f, z: %.3f, w: %.3f", 
              current_pose.orientation.x, current_pose.orientation.y, 
              current_pose.orientation.z, current_pose.orientation.w);
  RCLCPP_INFO(logger, "--------------------");

  // Create a new target pose by modifying the current one
	// Set a target Pose
	auto const target_pose = []{
	geometry_msgs::msg::Pose msg;
	msg.orientation.w = 0.707;
	msg.orientation.x = 0.707;
	msg.orientation.y = 0.0;
	msg.orientation.z = -0.0;
	msg.position.x = 0.034;
	msg.position.y = 0.050;
	msg.position.z = 0.075;
	return msg;
	}();
	move_group_interface.setPoseTarget(target_pose);

  // Create a plan
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    RCLCPP_INFO(logger, "Planning successful! Executing...");
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS gracefully
  rclcpp::shutdown();
  spinner.join(); // Wait for the thread to close
  return 0;
}