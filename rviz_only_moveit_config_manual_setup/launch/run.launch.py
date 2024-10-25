import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
            MoveItConfigsBuilder("mycobot_280", package_name="rviz_only_moveit_config_manual_setup")
            .planning_pipelines(pipelines=["ompl"])
            .robot_description(file_path="config/mycobot_280.urdf.xacro")
            .robot_description_semantic(file_path="config/mycobot_280.srdf")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .to_moveit_configs()
    )

    package = "hello_moveit_task_constructor"
    package_shared_path = get_package_share_directory(package)
    node = Node(
        package=package,
        executable=LaunchConfiguration("exe"),
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
        ],
    )

    arg = DeclareLaunchArgument(name="exe")
    return LaunchDescription([arg, node])
