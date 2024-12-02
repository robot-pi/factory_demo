#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <vector>
#include <memory>

class CartesianManipulatePanda : public rclcpp::Node
{
public:
  CartesianManipulatePanda()
      : Node("cartesian_manipulate_panda",
             rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // 构造函数中不调用 shared_from_this()
  }

  void init()
  {
    // 使用 rclcpp::Node::shared_from_this() 获取节点的 shared_ptr
    auto node_shared_ptr = shared_from_this();

    // 创建 MoveGroupInterface
    arm_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_shared_ptr, "panda_arm");

    // 设置规划时间和速度/加速度缩放因子
    arm_group_interface_->setPlanningTime(5.0);
    arm_group_interface_->setMaxVelocityScalingFactor(0.5);
    arm_group_interface_->setMaxAccelerationScalingFactor(0.5);

    // 设置末端执行器链接
    arm_group_interface_->setEndEffectorLink("panda_link8"); // 请根据您的机器人模型调整

    //arm_group_interface_->setGoalPositionTolerance(0.05); // 设置为5厘米
    //arm_group_interface_->setGoalOrientationTolerance(0.05); // 设置为0.05弧度

    // 创建 PlanningSceneMonitor
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_shared_ptr, "robot_description");

    // 启动 PlanningSceneMonitor
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor("/joint_states");
    planning_scene_monitor_->startWorldGeometryMonitor();

    // 创建订阅器，用于接收笛卡尔路径命令
    command_sub_ = create_subscription<geometry_msgs::msg::Vector3>(
        "cartesian_path_command", 10,
        std::bind(&CartesianManipulatePanda::commandCallback, this, std::placeholders::_1));
  }

private:
  void commandCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    // 等待机器人状态更新
    rclcpp::sleep_for(std::chrono::seconds(1));

    // 更新 PlanningScene
    planning_scene_monitor_->updateFrameTransforms();

    // 获取当前的 PlanningScene
    planning_scene_monitor::LockedPlanningSceneRW planning_scene(planning_scene_monitor_);

    // 获取当前的机器人状态
    auto &robot_state = planning_scene->getCurrentStateNonConst();

    // 获取末端执行器的当前位姿
    const std::string &end_effector_link = arm_group_interface_->getEndEffectorLink();
    const Eigen::Isometry3d &current_end_effector_state = robot_state.getGlobalLinkTransform(end_effector_link);

    // 将当前位姿转换为 Pose 消息
    geometry_msgs::msg::Pose current_pose_msg = tf2::toMsg(current_end_effector_state);

    // 输出当前位姿
    RCLCPP_INFO(this->get_logger(), "当前位姿 - x: %.3f, y: %.3f, z: %.3f",
                current_pose_msg.position.x, current_pose_msg.position.y, current_pose_msg.position.z);

    // 计算目标位姿
    geometry_msgs::msg::Pose target_pose = current_pose_msg;
    target_pose.position.x += msg->x;
    target_pose.position.y += msg->y;
    target_pose.position.z += msg->z;

    // 准备路点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(current_pose_msg); // 起始点
    waypoints.push_back(target_pose);      // 目标点

    // 计算笛卡尔路径
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.015;      // 末端执行器步长
    const double jump_threshold = 0.1; // 跳跃阈值
    double fraction = arm_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(this->get_logger(), "笛卡尔路径规划完成，完成率：%.2f%%", fraction * 100.0);

    // 如果路径规划成功，执行路径
    if (fraction > 0.1)
    {
      moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan;
      cartesian_plan.trajectory_ = trajectory;

      if (arm_group_interface_->execute(cartesian_plan) == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(this->get_logger(), "笛卡尔路径执行成功");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "笛卡尔路径执行失败");
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "未能规划出完整的笛卡尔路径");
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_interface_;
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr command_sub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  // 创建节点实例
  auto node = std::make_shared<CartesianManipulatePanda>();

  // 调用 init() 方法
  node->init();

  // 运行节点
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
