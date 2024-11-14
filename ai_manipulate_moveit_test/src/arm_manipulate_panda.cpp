#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
  
  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "arm_manipulate_panda",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("arm_manipulate_panda");

  // Create the MoveIt MoveGroup Interfaces
  // These interfaces are used to plan and execute movements, set target poses,
  // and perform other motion-related tasks for each respective part of the robot.
  // The use of auto allows the compiler to automatically deduce the type of variable.
  // Source: https://github.com/moveit/moveit2/blob/main/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h
  using moveit::planning_interface::MoveGroupInterface;
  auto panda_arm_group_interface = MoveGroupInterface(node, "panda_arm");

  // Specify a planning pipeline to be used for further planning 
  panda_arm_group_interface.setPlanningPipelineId("ompl");
  
  // Specify a planner to be used for further planning
  panda_arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  

  // Specify the maximum amount of time in seconds to use when planning
  panda_arm_group_interface.setPlanningTime(1.0);
  
  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  panda_arm_group_interface.setMaxVelocityScalingFactor(1.0);
  
  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  panda_arm_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", panda_arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", panda_arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", panda_arm_group_interface.getPlanningTime());

  // Set a target pose for the end effector of the arm 
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "panda_link0";
    msg.header.stamp = node->now(); 
    msg.pose.position.x = 0.5;
    msg.pose.position.y = -0.5;
    msg.pose.position.z = 0.3;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  panda_arm_group_interface.setPoseTarget(arm_target_pose);

  // Create a plan to that target pose
  // This will give us two things:
  // 1. Whether the planning was successful (stored in 'success')
  // 2. The actual motion plan (stored in 'plan')
  auto const [success, plan] = [&panda_arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(panda_arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Try to execute the movement plan if it was created successfully
  // If the plan wasn't successful, report an error
  // Execute the plan
  if (success)
  {
    panda_arm_group_interface.execute(plan);
  }
    else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
  
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}