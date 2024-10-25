#include <moveit_servo/servo_node.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  // 创建节点
  rclcpp::NodeOptions options;
  auto servo_node = std::make_shared<moveit_servo::ServoNode>(options);
  
  // Pause for the environment setup
  rclcpp::sleep_for(std::chrono::seconds(4));

  // 运行 ServoNode
  rclcpp::spin(servo_node->get_node_base_interface());

  rclcpp::shutdown();
}
