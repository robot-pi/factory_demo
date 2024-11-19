#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>
#include <string>
#include <sstream>

class GripperInputNode : public rclcpp::Node {
public:
  GripperInputNode() : Node("gripper_input_node") {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("gripper_command_input", 10);
  }

  void run() {
    while (rclcpp::ok()) {
      std::cout << "Enter gripper position and max effort (e.g., 0.035 10.0), or 'q' to quit: ";
      std::string input;
      std::getline(std::cin, input);

      if (input == "q") {
        break;
      }

      double position, max_effort;
      std::istringstream iss(input);
      if (!(iss >> position >> max_effort)) {
        std::cerr << "Invalid input. Please enter two numbers: position and max effort.\n";
        continue;
      }

      auto msg = std_msgs::msg::Float64MultiArray();
      msg.data = {position, max_effort};
      pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Published: position=%.3f, max_effort=%.3f", position, max_effort);
    }
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperInputNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}
