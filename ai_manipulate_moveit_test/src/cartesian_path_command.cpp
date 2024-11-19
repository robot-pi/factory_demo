#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <iostream>
#include <string>
#include <sstream>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cartesian_input_node");

  auto pub = node->create_publisher<geometry_msgs::msg::Vector3>("cartesian_path_command", 10);

  while (rclcpp::ok()) {
    std::cout << "Enter movement direction (x/y/z) and distance (meters), or 'q' to quit: ";
    std::string input;
    std::getline(std::cin, input);

    if (input == "q") {
      break;
    }

    char direction;
    double distance;
    std::istringstream iss(input);
    if (!(iss >> direction >> distance) || (direction != 'x' && direction != 'y' && direction != 'z')) {
      std::cout << "Invalid input." << std::endl;
      continue;
    }

    geometry_msgs::msg::Vector3 command;
    if (direction == 'x') {
      command.x = distance;
    } else if (direction == 'y') {
      command.y = distance;
    } else if (direction == 'z') {
      command.z = distance;
    }

    pub->publish(command);
    std::cout << "Command published: direction=" << direction << ", distance=" << distance << std::endl;
  }

  rclcpp::shutdown();
  return 0;
}
