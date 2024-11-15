# Author: Addison Sears-Collins
# Date: April 17, 2024
# Description: Launch a robotic arm in Gazebo (Classic)
import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  package_name_description = 'robotpi_demo_robot_description'
  package_name_gazebo = 'mycobot_gazebo'

  rviz_config_file_path = 'rviz/robotpi_demo_test.rviz'
  urdf_file_path = 'urdf/lite/car_lite.urdf.xacro' #'urdf/lite/car_lite.urdf.xacro' #'urdf/car.urdf.xacro' #'urdf/arm/mycobot280.urdf.xacro' #'urdf/arm/panda.urdf.xacro'
  gazebo_launch_file_path = 'launch'
  gazebo_models_path = 'models/factory'#factory
  world_file_path = 'worlds/factory.world'#'world/house_classic.world'#'world/factory.world'# world/empty_classic.world
  default_robot_name = 'car' #'mycobot_280' #'car' #'panda'
  map_file_path = 'maps/factory.yaml'

  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)
  pkg_share_slam = get_package_share_directory("robotpi_demo_robot_slam")
  pkg_share_navigation = get_package_share_directory('robotpi_demo_navigation')

  default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)
  default_urdf_model_path = os.path.join(pkg_share_description, urdf_file_path)
  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)
  map_path = os.path.join(pkg_share_slam, map_file_path)
  param_path = os.path.join(pkg_share_navigation, 'config', 'basic.yaml')

  headless = LaunchConfiguration('headless')
  robot_name = LaunchConfiguration('robot_name')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  urdf_model = LaunchConfiguration('urdf_model')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  # Set the default pose

  x = LaunchConfiguration('x')
  y = LaunchConfiguration('y')
  z = LaunchConfiguration('z')
  roll = LaunchConfiguration('roll')
  pitch = LaunchConfiguration('pitch')
  yaw = LaunchConfiguration('yaw')

  # Whether to use slam toolbox for mapping
  use_slam = LaunchConfiguration("use_slam")

  use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="False"
  )
  # Whether to use nav2 for planning
  use_plan = LaunchConfiguration("use_plan")

  use_plan_arg = DeclareLaunchArgument(
        "use_plan",
        default_value="True"
  )
  # Declare the launch arguments  
  declare_robot_name_cmd = DeclareLaunchArgument(
    name='robot_name',
    default_value=default_robot_name,
    description='The name for the robot')

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Display the Gazebo GUI if False, otherwise run in headless mode')

  declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start Gazebo')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
  
  declare_x_cmd = DeclareLaunchArgument(
    name='x',
    default_value='9.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='4.5',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='0.05',
    description='z component of initial position, meters')
    
  declare_roll_cmd = DeclareLaunchArgument(
    name='roll',
    default_value='0.0',
    description='roll angle of initial orientation, radians')

  declare_pitch_cmd = DeclareLaunchArgument(
    name='pitch',
    default_value='0.0',
    description='pitch angle of initial orientation, radians')

  declare_yaw_cmd = DeclareLaunchArgument(
    name='yaw',
    default_value='1.5708',
    description='yaw angle of initial orientation, radians')


  set_env_vars_resources = AppendEnvironmentVariable(
    'GAZEBO_MODEL_PATH',
    gazebo_models_path)


  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Start the SLAM Toolbox (only if use_slam is true)
  slam = IncludeLaunchDescription(
      os.path.join(
          pkg_share_slam,
          "launch",
          "slam.launch.py"
      ),
      condition=IfCondition(use_slam)
  )

  # Start the Navigation2 stack (only if use_plan is true)
  nav2_plan = IncludeLaunchDescription(
      os.path.join(
          pkg_share_navigation,
          "launch",
          "navigation.launch.py"
      ),
      condition=IfCondition(use_plan),
      launch_arguments={
          'map': map_path,
          'params_file': param_path,
          'use_sim_time': use_sim_time,
          'x': x,
          'y': y,
          'z': z,
          'roll': roll,
          'pitch': pitch,
          'yaw': yaw
      }.items(),
  )

  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  robot_description_content = ParameterValue(Command(['xacro ', urdf_model]), value_type=str)
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])
  
  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time, 
      'robot_description': robot_description_content}])
  

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(PythonExpression([use_rviz, ' and not ', use_slam])),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file],
    parameters=[{'use_sim_time': use_sim_time, }]
    )
  
  start_rviz_slam = Node(
      condition=IfCondition(PythonExpression([use_rviz, ' and ', use_slam])),
      package="rviz2",
      executable="rviz2",
      arguments=["-d", os.path.join(
              pkg_share_slam,
              "rviz",
              "mapping.rviz"
          )
      ],
      output="screen",
      parameters=[{"use_sim_time": True}],
  )  
    
  # Spawn the robot
  start_gazebo_ros_spawner_cmd = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity', robot_name,
      '-topic', "robot_description",  
      '-x', x,
      '-y', y,
      '-z', z,
      '-R', roll,
      '-P', pitch,
      '-Y', yaw
      ],
    output='screen')  
  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_robot_name_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_urdf_model_path_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)


  ld.add_action(declare_x_cmd)
  ld.add_action(declare_y_cmd)
  ld.add_action(declare_z_cmd)
  ld.add_action(declare_roll_cmd)
  ld.add_action(declare_pitch_cmd)
  ld.add_action(declare_yaw_cmd)  


  # Add any actions
  ld.add_action(set_env_vars_resources)
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)

  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)
  
  ld.add_action(start_gazebo_ros_spawner_cmd)
  ld.add_action(TimerAction(period=5.0, actions=[start_rviz_cmd]))

  ld.add_action(use_slam_arg)
  ld.add_action(slam)
  ld.add_action(start_rviz_slam)

  ld.add_action(use_plan_arg)
  ld.add_action(nav2_plan)

  return ld