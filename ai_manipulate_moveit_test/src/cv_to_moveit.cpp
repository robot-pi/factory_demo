#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>

struct Params {
  std::string object_name;
  std::string object_reference_frame;
  std::vector<double> object_pose;          // {x, y, z, qx, qy, qz, qw}
  std::vector<double> object_dimensions;    // {height, radius} for a cylinder
};

// 将位姿向量转换为 Pose
geometry_msgs::msg::Pose vectorToPose(const std::vector<double>& vec) {
  geometry_msgs::msg::Pose pose;
  pose.position.x = vec[0];
  pose.position.y = vec[1];
  pose.position.z = vec[2];
  pose.orientation.x = vec[3];
  pose.orientation.y = vec[4];
  pose.orientation.z = vec[5];
  pose.orientation.w = vec[6];
  return pose;
}

// 创建 CollisionObject
moveit_msgs::msg::CollisionObject createObject(const Params& params) {
  geometry_msgs::msg::Pose pose = vectorToPose(params.object_pose);
  moveit_msgs::msg::CollisionObject object;
  object.id = params.object_name;
  object.header.frame_id = params.object_reference_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { params.object_dimensions.at(0), params.object_dimensions.at(1) };
  pose.position.z += 0.5 * params.object_dimensions[0];
  object.primitive_poses.push_back(pose);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

// 主节点
class CollisionObjectPublisher : public rclcpp::Node {
public:
  CollisionObjectPublisher()
      : Node("collision_object_publisher") {
    // 初始化客户端和参数
    client_ = this->create_client<moveit_msgs::srv::ApplyPlanningScene>("apply_planning_scene");

    Params params;
    params.object_name = "grub_object_1";
    params.object_reference_frame = "world";
    params.object_pose = {0.3, -0.15, 0, 0.0, 0.0, 0.0, 1.0};
    params.object_dimensions = {1, 0.03};

    // 创建并设置 CollisionObject
    collision_object_ = createObject(params);

    // 定时器定时发送请求
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&CollisionObjectPublisher::updatePlanningScene, this));
  }

private:
  void updatePlanningScene() {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(this->get_logger(), "Service not available, waiting...");
      return;
    }

    auto request = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object_);
    planning_scene.is_diff = true;
    request->scene = planning_scene;

    auto future = client_->async_send_request(request);

    future.wait();

    if (future.get()->success) {
      RCLCPP_INFO(this->get_logger(), "Successfully updated planning scene");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to update planning scene");
    }
  }

  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr client_;
  moveit_msgs::msg::CollisionObject collision_object_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionObjectPublisher>());
  rclcpp::shutdown();
  return 0;
}
