#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <string>
#include <algorithm>

class JointStateFilter : public rclcpp::Node
{
public:
    JointStateFilter() : Node("joint_state_filter")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&JointStateFilter::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/filtered_joint_states", 10);

        main_joints_ = {"link1_to_link2", "link2_to_link3", "link3_to_link4", 
                        "link4_to_link5", "link5_to_link6", "link6_to_link6flange", 
                        "gripper_controller"};
    }

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        sensor_msgs::msg::JointState filtered_msg;
        filtered_msg.header = msg->header;

        for (size_t i = 0; i < msg->name.size(); ++i)
        {
            if (std::find(main_joints_.begin(), main_joints_.end(), msg->name[i]) != main_joints_.end())
            {
                filtered_msg.name.push_back(msg->name[i]);
                filtered_msg.position.push_back(msg->position[i]);
                if (!msg->velocity.empty()) filtered_msg.velocity.push_back(msg->velocity[i]);
                if (!msg->effort.empty()) filtered_msg.effort.push_back(msg->effort[i]);
            }
        }

        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    std::vector<std::string> main_joints_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateFilter>());
    rclcpp::shutdown();
    return 0;
}