import os
import yaml
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription(
         [OpaqueFunction(function=launch_setup)]
    )

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            yaml_content = yaml.safe_load(file)
            print(f"Successfully loaded YAML from {absolute_file_path}")
            return yaml_content
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name="rviz_only_moveit_config_manual_setup")
        .robot_description(file_path="config/mycobot_280.urdf.xacro")
        .robot_description_semantic(file_path="config/mycobot_280.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        #.planning_pipelines(
        #    pipelines=["ompl", "pilz_industrial_motion_planner"],
        #)
        .to_moveit_configs()
    )
    rviz_config_file = (
    get_package_share_directory("rviz_only_moveit_config_manual_setup") + "/rviz/hello_moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("rviz_only_moveit_config_manual_setup"),
        "config",
        "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package = "controller_manager",
        executable = "ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grip_action_controller", "-c", "/controller_manager"],
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("mycobot_280_move_servo_package", "config/mycobot_280_simulated_config.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    print(f"Servo parameters: {servo_params}")

    servo_node = Node(
        package="mycobot_280_move_servo_package",
        executable="servo_node_main",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            servo_params,
        ],
        output="screen",   
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/base_link", "frame_id": "/world"}],
            ),
        ],
        output="screen",
    )

    nodes_to_start = [
        rviz_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        servo_node,
        move_group_node,
        container,
    ]

    return nodes_to_start