import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():

  package_name_description = 'robotpi_demo_robot_description'
  package_name_gazebo = 'mycobot_gazebo'

  rviz_config_file_path = 'rviz/robotpi_demo3.rviz'
  urdf_file_path = 'urdf/arm/panda.urdf.xacro' #'urdf/lite/car_lite.urdf.xacro' #'urdf/car.urdf.xacro' #'urdf/arm/mycobot280.urdf.xacro' #'urdf/arm/panda.urdf.xacro'
  gazebo_launch_file_path = 'launch'
  gazebo_models_path = 'models/factory'#factory
  world_file_path = 'worlds/burger_assemble.world'#'world/house_classic.world'#'world/factory.world'# world/empty_classic.world burger_assemble
  default_robot_name = 'panda' #'mycobot_280' #'car' #'panda'

  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros') 
  pkg_share_description = FindPackageShare(package=package_name_description).find(package_name_description)
  pkg_share_gazebo = FindPackageShare(package=package_name_gazebo).find(package_name_gazebo)

  default_rviz_config_path = os.path.join(pkg_share_description, rviz_config_file_path)
  default_urdf_model_path = os.path.join(pkg_share_description, urdf_file_path)
  gazebo_launch_file_path = os.path.join(pkg_share_gazebo, gazebo_launch_file_path)
  gazebo_models_path = os.path.join(pkg_share_gazebo, gazebo_models_path)
  world_path = os.path.join(pkg_share_gazebo, world_file_path)

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
    default_value='0.0',
    description='x component of initial position, meters')

  declare_y_cmd = DeclareLaunchArgument(
    name='y',
    default_value='0.0',
    description='y component of initial position, meters')
    
  declare_z_cmd = DeclareLaunchArgument(
    name='z',
    default_value='1.015',
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
    default_value='0.0',
    description='yaw angle of initial orientation, radians')


  set_env_vars_resources = AppendEnvironmentVariable(
    'GAZEBO_MODEL_PATH',
    gazebo_models_path)

  start_panda_arm_controller_cmd = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "panda_arm_controller",
      "--controller-manager",
      "/controller_manager"
    ]
  )  

  start_panda_hand_controller_cmd = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "panda_hand_controller",
      "--controller-manager",
      "/controller_manager"
    ],
  )  

  # Launch joint state broadcaster
  start_joint_state_broadcaster_cmd = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "joint_state_broadcaster",
      "--controller-manager",
      "/controller_manager"
    ]
  )  

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

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file],
    parameters=[{'use_sim_time': use_sim_time, }]
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
# 1. 设置环境变量
  ld.add_action(set_env_vars_resources)

  # 2. 启动 Gazebo 服务器和客户端
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(TimerAction(period=3.0, actions=[start_gazebo_client_cmd]))

  # 3. 启动 robot_state_publisher，延迟 5 秒以确保 Gazebo 服务器完全启动
  ld.add_action(TimerAction(period=3.0, actions=[start_robot_state_publisher_cmd]))

  # 4. 启动控制器，分别延迟启动以确保 robot_state_publisher 已初始化
  ld.add_action(TimerAction(period=5.0, actions=[start_panda_arm_controller_cmd]))
  ld.add_action(TimerAction(period=6.0, actions=[start_panda_hand_controller_cmd]))
  ld.add_action(TimerAction(period=7.0, actions=[start_joint_state_broadcaster_cmd]))

  # 5. 启动机器人模型（spawn entity），延迟 2 秒以确保控制器已启动
  ld.add_action(TimerAction(period=10.0, actions=[start_gazebo_ros_spawner_cmd]))

  # 6. 启动 RViz，最后延迟启动以确保所有系统都已准备好
  #ld.add_action(TimerAction(period=13.0, actions=[start_rviz_cmd]))


  return ld