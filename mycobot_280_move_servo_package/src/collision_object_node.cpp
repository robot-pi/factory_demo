#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_world.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

class CollisionObjectPublisher : public rclcpp::Node
{
public:
  CollisionObjectPublisher() : Node("collision_object_node")
  {
    // 创建发布器，用于发布碰撞物体到 planning_scene 话题
    scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 10);

    // 创建并发布碰撞物体
    publishCollisionObject();
  }

private:
  // 碰撞物体发布器
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_pub_;

  void publishCollisionObject()
  {
    // 创建碰撞物体
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = "base_link";
    object.id = "box";

    // 定义box的形状和尺寸
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { 0.015, 0.015, 0.15 };

    // 设置box的位置
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.15;
    box_pose.position.y = 0.15;
    box_pose.position.z = 0.0;

    // 将box添加到碰撞物体中
    object.primitives.push_back(box);
    object.primitive_poses.push_back(box_pose);
    object.operation = object.ADD;

    // 创建 PlanningScene 消息并将碰撞物体添加到世界中
    moveit_msgs::msg::PlanningSceneWorld psw;
    psw.collision_objects.push_back(object);

    moveit_msgs::msg::PlanningScene ps;
    ps.is_diff = true;  // 表示这是一个增量更新
    ps.world = psw;

    // 发布碰撞物体到 planning_scene 话题
    RCLCPP_INFO(this->get_logger(), "Publishing collision object to the planning scene...");
    scene_pub_->publish(ps);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionObjectPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
