#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

/**
 * @brief The main function that starts our program.
 *
 * This function sets up our ROS 2 environment and prepares it for robot control.
 *
 * @param argc The number of input arguments our program receives.
 * @param argv The list of input arguments our program receives.
 * @return int A number indicating if our program finished successfully (0) or not.
 */
int main(int argc, char * argv[])
{
  // Start up ROS 2
  rclcpp::init(argc, argv);
   
  // Creates a node named "hello_moveit". The node is set up to automatically
  // handle any settings (parameters) we might want to change later without editing the code.
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
 
  // Creates a "logger" that we can use to print out information or error messages
  // as our program runs.
  auto const logger = rclcpp::get_logger("hello_moveit");
 
  // Create the MoveIt MoveGroup Interfaces
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group_interface = MoveGroupInterface(node, "arm");
 
  // Specify a planning pipeline to be used for further planning 
  arm_group_interface.setPlanningPipelineId("ompl");
   
  // Specify a planner to be used for further planning
  arm_group_interface.setPlannerId("RRTConnectkConfigDefault");  
 
  // Specify the maximum amount of time in seconds to use when planning
  arm_group_interface.setPlanningTime(1.0);
   
  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  arm_group_interface.setMaxVelocityScalingFactor(1.0);
   
  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  arm_group_interface.setMaxAccelerationScalingFactor(1.0);
 
  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", arm_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", arm_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", arm_group_interface.getPlanningTime());
 
  // Set a target pose for the end effector of the arm 
  auto const arm_target_pose = [&node]{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = node->now(); 
    msg.pose.position.x = 0.125;
    msg.pose.position.y = -0.125;
    msg.pose.position.z = 0.05;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 1.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    return msg;
  }();
  arm_group_interface.setPoseTarget(arm_target_pose);
 
  // Create a plan to that target pose
  auto const [success, plan] = [&arm_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(arm_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
 
  // Try to execute the movement plan if it was created successfully
  if (success)
  {
    arm_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }
/*
   // Setup Planning Scene (Add a cylindrical object)
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  // Define the collision object (cylinder)
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.id = "cylinder";
  collision_object.header.frame_id = "base_link";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions = { 0.1, 0.01 };  // Height and radius

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.125;
  pose.position.y = -0.125;
  pose.position.z = 0.05;  // Adjust Z position to prevent collision with ground
  pose.orientation.w = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);
  collision_object.operation = collision_object.ADD;

  // Apply the object to the planning scene
  planning_scene_interface.applyCollisionObject(collision_object);
   */
  // Shut down ROS 2 cleanly when we're done
  rclcpp::shutdown();
  return 0;
}
