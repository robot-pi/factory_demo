#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>

int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>("cartesian_path_demo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto const logger = rclcpp::get_logger("cartesian_path_demo");

  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");

  // Set planning pipeline, planner, planning time, and scaling factors
  arm_group_interface.setPlanningPipelineId("ompl");
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");
  arm_group_interface.setPlanningTime(5.0);
  arm_group_interface.setMaxVelocityScalingFactor(0.5);
  arm_group_interface.setMaxAccelerationScalingFactor(0.5);

  // Set the end-effector link as the reference frame for Cartesian planning
  arm_group_interface.setEndEffectorLink("gripper_base");  // Replace "end_effector_link" with your actual end-effector link name

  // Set the initial pose as the starting point for Cartesian path planning
  geometry_msgs::msg::PoseStamped start_pose = arm_group_interface.getCurrentPose("gripper_base");
  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(start_pose.pose);

  // Define a target pose with Cartesian offset relative to the end-effector's frame
  geometry_msgs::msg::Pose target_pose = start_pose.pose;
  target_pose.position.y -= 0.01;  // Move 5 cm in x direction
  waypoints.push_back(target_pose);

  // Plan the Cartesian path, specifying a step size and jump threshold
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.001;  // Small step for smooth path
  const double jump_threshold = 0.0;  // Disable jump threshold

  // Compute the Cartesian path
  double fraction = arm_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Log the result of the path planning
  RCLCPP_INFO(logger, "Cartesian path (%.2f%% achieved)", fraction * 100.0);

  // Execute the Cartesian path if it was planned successfully
  if (fraction > 0.9)  // Success threshold
  {
    moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
    cartesian_plan.trajectory_ = trajectory;
    arm_group_interface.execute(cartesian_plan);
    RCLCPP_INFO(logger, "Cartesian path execution successful");
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to plan a full Cartesian path");
  }

  rclcpp::shutdown();
  return 0;
}
