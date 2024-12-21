#include <rclcpp/rclcpp.hpp>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// tau = 1 rotation in radiants
const double tau = 2 * M_PI;

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint2"; // right finger
    posture.joint_names[1] = "panda_finger_joint1"; // left finger

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.035;
    posture.points[0].positions[1] = -0.035;
    posture.points[0].time_from_start = rclcpp::sleep_for(std::chrono::milliseconds(500));
;

}


void closedGripper(trajectory_msgs::JointTrajectory& posture)
{

    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint2";
    posture.joint_names[1] = "panda_finger_joint1";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.055;
    posture.points[0].positions[1] = -0.055;
    posture.points[0].time_from_start = rclcpp::sleep_for(std::chrono::milliseconds(500));
;


}

void pick(moveit::planning_interface::MoveGroupInterface& move_group)
{
    std::vector<moveit_msgs::msg::Grasp> grasps;
    grasps.resize(1);

    // Grasp pose
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;


    // Pick Test
    orientation.setRPY(0.180, 1.567, 0.181);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.299;
    grasps[0].grasp_pose.pose.position.y = -0.5;
    grasps[0].grasp_pose.pose.position.z = 0.065;

    // Pre-grasp approach
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    //Direction is set as negative z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    //Direction is set as positive z axis 
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;



    // we need to open the gripper. We will define a function for that
    // openGripper(grasps[0].pre_grasp_posture);

    // When it grasps it needs to close the gripper
    // closedGripper(grasps[0].grasp_posture);

    // Set support surface as cube
    move_group.setSupportSurfaceName("table1");

    // Call pick to pick up the object using the grasps given
    // move_group.pick("object", grasps);

    
}

void place(moveit::planning_interface::MoveGroupInterface& group)
{
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;


    // Test
    orientation.setRPY(0, 0, tau / 4);  // A quarter turn about the z-axis
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);
    
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 1;
    place_location[0].place_pose.pose.position.z = 0.6;

    // Setting pre-place approach
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    // Direction is set as negative z axis
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    // Direction is set as negative y axis
    place_location[0].post_place_retreat.direction.vector.x = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.1;
    place_location[0].post_place_retreat.desired_distance = 0.25;

    // openGripper(place_location[0].post_place_posture);

    // Set support surface as table 2
    group.setSupportSurfaceName("table2");
    
    // Call place to palce the object using the place location given
    // group.place("object", place_location);

}

void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the wood cube
    collision_objects[0].id = "wood_cube";
    collision_objects[0].header.frame_id = "panda_link0";

    // Define primitive dimension, position of the cube
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.05;
    collision_objects[0].primitives[0].dimensions[1] = 0.05;
    collision_objects[0].primitives[0].dimensions[2] = 0.05;
    // pose of cube
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.3;
    collision_objects[0].primitive_poses[0].position.y = -0.5;
    collision_objects[0].primitive_poses[0].position.z = 0.0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // Add tabe 1 to the scene
    collision_objects[0].operation = collision_objects[0].ADD;


    // // Add the second table
    // collision_objects[1].id = "table2";
    // collision_objects[1].header.frame_id = "base_link";

    // // Define primitive dimension, position of the table 2
    // collision_objects[1].primitives.resize(1);
    // collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[1].primitives[0].dimensions.resize(3);
    // collision_objects[1].primitives[0].dimensions[0] = 0.2;
    // collision_objects[1].primitives[0].dimensions[1] = 0.4;
    // collision_objects[1].primitives[0].dimensions[2] = 0.4;
    // // pose of table 2
    // collision_objects[1].primitive_poses.resize(1);
    // collision_objects[1].primitive_poses[0].position.x = 0;
    // collision_objects[1].primitive_poses[0].position.y = 1;
    // collision_objects[1].primitive_poses[0].position.z = 0.2;
    // collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // // Add tabe 2 to the scene
    // collision_objects[1].operation = collision_objects[1].ADD;

    // // Add the object to be picked
    // collision_objects[2].id = "object";
    // collision_objects[2].header.frame_id = "base_link";

    // // Define primitive dimension, position of the object
    // collision_objects[2].primitives.resize(1);
    // collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
    // collision_objects[2].primitives[0].dimensions.resize(3);
    // collision_objects[2].primitives[0].dimensions[0] = 0.02;
    // collision_objects[2].primitives[0].dimensions[1] = 0.02;
    // collision_objects[2].primitives[0].dimensions[2] = 0.2;
    // // pose of object
    // collision_objects[2].primitive_poses.resize(1);
    // collision_objects[2].primitive_poses[0].position.x = 1;
    // collision_objects[2].primitive_poses[0].position.y = 0;
    // collision_objects[2].primitive_poses[0].position.z = 0.5;
    // collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // // Add tabe 2 to the object
    // collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);

}

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create the ROS 2 node
    auto node = rclcpp::Node::make_shared("panda_pick_and_place");

    // Create a planning scene interface and a move group interface
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group(node, "panda_arm");

    // Set planning time for the group
    group.setPlanningTime(45.0);

    // Add the object to the planning scene
    RCLCPP_INFO(node->get_logger(), "Adding collision object to the planning scene");
    addCollisionObject(planning_scene_interface);

    // Wait for the scene to update
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Pick the object
    RCLCPP_INFO(node->get_logger(), "Picking the object");
    pick(group);

    // Wait for the pick operation to complete
    rclcpp::sleep_for(std::chrono::seconds(1));

    // Place the object
    RCLCPP_INFO(node->get_logger(), "Placing the object");
    // place(group);

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
