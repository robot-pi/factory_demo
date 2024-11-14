#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  // 初始化 ROS 2
  rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
    "gripper_manipulate",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("gripper_manipulate");
  // 创建 MoveGroupInterface，用于控制夹具
  using moveit::planning_interface::MoveGroupInterface;
  auto gripper_group_interface = MoveGroupInterface(node, "gripper");

    // Specify a planning pipeline to be used for further planning 
  gripper_group_interface.setPlanningPipelineId("ompl");
  
  // Specify a planner to be used for further planning
  gripper_group_interface.setPlannerId("RRTConnectkConfigDefault");  

  // Specify the maximum amount of time in seconds to use when planning
  gripper_group_interface.setPlanningTime(1.0);
  
  // Set a scaling factor for optionally reducing the maximum joint velocity. Allowed values are in (0,1].
  gripper_group_interface.setMaxVelocityScalingFactor(1.0);
  
  //  Set a scaling factor for optionally reducing the maximum joint acceleration. Allowed values are in (0,1].
  gripper_group_interface.setMaxAccelerationScalingFactor(1.0);

  // Display helpful logging messages on the terminal
  RCLCPP_INFO(logger, "Planning pipeline: %s", gripper_group_interface.getPlanningPipelineId().c_str());    
  RCLCPP_INFO(logger, "Planner ID: %s", gripper_group_interface.getPlannerId().c_str());
  RCLCPP_INFO(logger, "Planning time: %.2f", gripper_group_interface.getPlanningTime());

  // 设置夹具的目标关节值 (直接写入数值)
  std::vector<double> gripper_open_pose = {-0.558};  // 这里假设一个单关节夹具，0.04 表示夹具打开的目标值
  gripper_group_interface.setJointValueTarget(gripper_open_pose);

  // 执行夹具打开的动作
  bool success = (gripper_group_interface.move() == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "Gripper opened successfully.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to open gripper.");
  }

  // 关闭 ROS 2
  rclcpp::shutdown();
  return 0;
}