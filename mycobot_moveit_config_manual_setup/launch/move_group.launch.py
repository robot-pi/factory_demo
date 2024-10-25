# Author: Addison Sears-Collins
# Date: July 31, 2024
# Description: Launch MoveIt 2 for the myCobot robotic arm
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessStart
import xacro
 
 
def generate_launch_description():
 
    # Constants for paths to different files and folders
    package_name_gazebo = 'mycobot_gazebo'
    package_name_moveit_config = 'mycobot_moveit_config_manual_setup'
 
    # Set the path to different files and folders
    pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
    pkg_share_moveit_config = FindPackageShare(package=package_name_moveit_config).find(package_name_moveit_config)
 
    # Paths for various configuration files
    urdf_file_path = 'urdf/ros2_control/classic_gazebo/mycobot_280.urdf.xacro'
    srdf_file_path = 'config/mycobot_280.srdf'
    moveit_controllers_file_path = 'config/moveit_controllers.yaml'
    joint_limits_file_path = 'config/joint_limits.yaml'
    kinematics_file_path = 'config/kinematics.yaml'
    pilz_cartesian_limits_file_path = 'config/pilz_cartesian_limits.yaml'
    initial_positions_file_path = 'config/initial_positions.yaml'
    rviz_config_file_path = 'rviz/move_group.rviz'
 
    # Set the full paths
    urdf_model_path = os.path.join(pkg_share_gazebo, urdf_file_path)
    srdf_model_path = os.path.join(pkg_share_moveit_config, srdf_file_path)
    moveit_controllers_file_path = os.path.join(pkg_share_moveit_config, moveit_controllers_file_path)
    joint_limits_file_path = os.path.join(pkg_share_moveit_config, joint_limits_file_path)
    kinematics_file_path = os.path.join(pkg_share_moveit_config, kinematics_file_path)
    pilz_cartesian_limits_file_path = os.path.join(pkg_share_moveit_config, pilz_cartesian_limits_file_path)
    initial_positions_file_path = os.path.join(pkg_share_moveit_config, initial_positions_file_path)
    rviz_config_file = os.path.join(pkg_share_moveit_config, rviz_config_file_path)
 
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
 
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RViz')
 
    # Load the robot configuration
    # Typically, you would also have this line in here: .robot_description(file_path=urdf_model_path)
    # Another launch file is launching the robot description.
    moveit_config = (
        MoveItConfigsBuilder("mycobot_280", package_name=package_name_moveit_config)
        .trajectory_execution(file_path=moveit_controllers_file_path)
        # only for rviz
        #.robot_description(file_path=urdf_model_path) # add urdf file for moveit
        .robot_description_semantic(file_path=srdf_model_path)# srdf file for moveit
        .joint_limits(file_path=joint_limits_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"], # , "stomp", "ompl"
            default_planning_pipeline="pilz_industrial_motion_planner"
        )
        .planning_scene_monitor(
            publish_robot_description=False, # when using gazebo simulation, moveit needs to load urdf file but does't need to publish urdf
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .pilz_cartesian_limits(file_path=pilz_cartesian_limits_file_path)
        .to_moveit_configs()
    )
    
    # 修改moveit_config字典以设置joint state topic
    moveit_config_dict = moveit_config.to_dict()
    if 'move_group' not in moveit_config_dict:
        moveit_config_dict['move_group'] = {}
    moveit_config_dict['move_group']['joint_states_topic'] = '/filtered_joint_states'
    # Start the actual move_group node/action server
    start_move_group_node_cmd = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            #moveit_config.to_dict(),
            moveit_config_dict,
            {'use_sim_time': use_sim_time},
            {'start_state': {'content': initial_positions_file_path}},
        ],
    )
 
    # RViz
    start_rviz_node_cmd = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time}
        ],
    )

    # 添加 joint_state_filter 节点
    start_joint_state_filter_node_cmd = Node(
        package='mycobot_moveit_config_manual_setup',
        executable='joint_state_filter',
        name='joint_state_filter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_topic': '/joint_states'},
            {'output_topic': '/filtered_joint_states'},
        ],
    )
        # 确保 joint_state_filter 在 move_group 之前启动
    joint_state_filter_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=start_joint_state_filter_node_cmd,
            on_start=[start_move_group_node_cmd]
        )
    )

    '''
    # 添加 hello_moveit 节点
    start_hello_moveit_node_cmd = Node(
    package='mycobot_control',
    executable='hello_moveit',
    name='hello_moveit',
    output='screen',
    parameters=[
        moveit_config.to_dict(),
        {'use_sim_time': use_sim_time}
    ],
    )
'''

     
    exit_event_handler = RegisterEventHandler(
        condition=IfCondition(use_rviz),
        event_handler=OnProcessExit(
            target_action=start_rviz_node_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
        ),
    )
     
    # Create the launch description and populate
    ld = LaunchDescription()
 
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
 
    ld.add_action(start_joint_state_filter_node_cmd)
    ld.add_action(joint_state_filter_event)
    # Add any actions
    #ld.add_action(start_move_group_node_cmd)
    ld.add_action(start_rviz_node_cmd)
     
    # 添加 joint_state_filter 节点

    #ld.add_action(start_hello_moveit_node_cmd)

    # Clean shutdown of RViz
    ld.add_action(exit_event_handler)
 
    return ld
