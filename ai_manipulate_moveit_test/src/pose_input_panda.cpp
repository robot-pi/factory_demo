#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.hpp>
#include <iostream>
#include <sstream>

namespace {
// Transform a vector of numbers into a 3D position and orientation
Eigen::Isometry3d vectorToEigen(const std::vector<double>& values) {
  return Eigen::Translation3d(values[0], values[1], values[2]) *
         Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ());
}

// Convert a vector of numbers to a geometry_msgs::msg::Pose
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& values) {
  return tf2::toMsg(vectorToEigen(values));
};
}  // namespace

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto const node = std::make_shared<rclcpp::Node>("pose_input_node");

    auto pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

    while (rclcpp::ok()) {
    std::cout << "Please enter target position and orientation! (x y z roll pitch yaw in radians, 'q' to quit)" << std::endl;

    std::string target_input;
    std::getline(std::cin, target_input);

    if (target_input == "q") {
      break;
    }

    std::istringstream iss(target_input);
    std::vector<double> values(6);  // x, y, z, roll, pitch, yaw
    if (!(iss >> values[0] >> values[1] >> values[2] >> values[3] >> values[4] >> values[5])) {
      std::cerr << "Invalid input format! Please enter 6 values: x y z roll pitch yaw." << std::endl;
      continue;
    }

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "panda_link0";
    target_pose.header.stamp = node->now();
    target_pose.pose = vectorToPose(values);

    pub->publish(target_pose);
    std::cout << "Target pose published." << std::endl;
  }

  rclcpp::shutdown();
  return 0;

}