#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

class CartesianPathNode : public rclcpp::Node {
public:
  CartesianPathNode()
      : Node("cartesian_path_node",
             rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
        arm_group_interface_(std::shared_ptr<rclcpp::Node>(this), "panda_arm") {
    // 配置 MoveGroupInterface
    arm_group_interface_.setPlanningTime(5.0);
    arm_group_interface_.setMaxVelocityScalingFactor(0.5);
    arm_group_interface_.setMaxAccelerationScalingFactor(0.5);

    RCLCPP_INFO(get_logger(), "End effector link: %s", arm_group_interface_.getEndEffectorLink().c_str());
    arm_group_interface_.setEndEffectorLink("panda_link8");

    // 创建订阅器，订阅 "cartesian_path_command" 话题
    command_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "cartesian_path_command", 10,
        std::bind(&CartesianPathNode::commandCallback, this, std::placeholders::_1));
  }

private:
  void commandCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
    // 获取末端执行器的当前位姿
    geometry_msgs::msg::PoseStamped start_pose = arm_group_interface_.getCurrentPose();

    // 准备笛卡尔路径的路点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose.pose);

    // 根据接收到的指令计算目标位姿
    geometry_msgs::msg::Pose target_pose = start_pose.pose;
    target_pose.position.x += msg->x;
    target_pose.position.y += msg->y;
    target_pose.position.z += msg->z;
    waypoints.push_back(target_pose);

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.005;  // 末端执行器步长
    const double jump_threshold = 0.0;
    double fraction = arm_group_interface_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0);

    // 如果规划成功，执行笛卡尔路径
    if (fraction > 0.9) {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;
      if (arm_group_interface_.execute(cartesian_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(get_logger(), "Cartesian path execution successful");
      } else {
        RCLCPP_ERROR(get_logger(), "Cartesian path execution failed");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to plan a full Cartesian path");
    }
  }

  moveit::planning_interface::MoveGroupInterface arm_group_interface_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr command_sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CartesianPathNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
