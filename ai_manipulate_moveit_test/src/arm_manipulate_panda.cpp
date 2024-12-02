#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class ArmManipulationNode : public rclcpp::Node {
public:
  ArmManipulationNode()
      : Node("arm_manipulate_panda",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),arm_group_interface_(std::shared_ptr<rclcpp::Node>(this), "panda_arm") {
    // Configure the arm group interface
    arm_group_interface_.setPlanningPipelineId("ompl");
    arm_group_interface_.setPlannerId("RRTConnectkConfigDefault");
    arm_group_interface_.setPlanningTime(1.0);
    arm_group_interface_.setMaxVelocityScalingFactor(1.0);
    arm_group_interface_.setMaxAccelerationScalingFactor(1.0);

    RCLCPP_INFO(get_logger(), "Initialized arm manipulation node.");

    // Print all available groups and the current end-effector link
  auto groups = arm_group_interface_.getJointModelGroupNames();
  RCLCPP_INFO(get_logger(), "Available groups:");
  for (const auto& group : groups) {
    RCLCPP_INFO(get_logger(), "  - %s", group.c_str());
  }

  RCLCPP_INFO(get_logger(), "End Effector Link: %s", arm_group_interface_.getEndEffectorLink().c_str());

    // Create subscription to the "target_pose" topic
    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("target_pose", 10, std::bind(&ArmManipulationNode::poseCallback, this, std::placeholders::_1));
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Received target pose!");

    // Set the received pose as the target for the MoveGroupInterface
    arm_group_interface_.setPoseTarget(*msg);

    // Plan and execute the motion
    auto const [success, plan] = [&] {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto success = static_cast<bool>(arm_group_interface_.plan(plan));
      return std::make_pair(success, plan);
    }();

    if (success) {
      arm_group_interface_.execute(plan);
      RCLCPP_INFO(get_logger(), "Motion execution succeeded.");
    } else {
      RCLCPP_ERROR(get_logger(), "Motion planning failed!");
    }
  }

  moveit::planning_interface::MoveGroupInterface arm_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmManipulationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
