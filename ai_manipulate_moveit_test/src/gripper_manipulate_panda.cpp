#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using GripperCommand = control_msgs::action::GripperCommand;

class GripperControlNode : public rclcpp::Node {
public:
  GripperControlNode() : Node("gripper_control_node") {
    client_ = rclcpp_action::create_client<GripperCommand>(this, "/panda_hand_controller/gripper_cmd");
    sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        "gripper_command_input", 10,
        std::bind(&GripperControlNode::commandCallback, this, std::placeholders::_1));
  }

private:
  void commandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 2) {
      RCLCPP_ERROR(get_logger(), "Invalid command received. Expected 2 values: position and max effort.");
      return;
    }

    if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available!");
      return;
    }

    auto goal_msg = GripperCommand::Goal();
    goal_msg.command.position = msg->data[0];
    goal_msg.command.max_effort = msg->data[1];

    auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<GripperCommand>::WrappedResult &result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rclcpp::get_logger("GripperControlNode"), "Goal succeeded!");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("GripperControlNode"), "Goal failed!");
      }
    };

    client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "Sent goal: position=%.3f, max_effort=%.3f", goal_msg.command.position, goal_msg.command.max_effort);
  }

  rclcpp_action::Client<GripperCommand>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
