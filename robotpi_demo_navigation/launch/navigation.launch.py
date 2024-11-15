import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    MODEL = 'basic'
    param_file_name = MODEL + '.yaml'
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            get_package_share_directory('robotpi_demo_robot_slam'),
            'maps',
            'factory.yaml'))
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('robotpi_demo_navigation'),
            'config',
            param_file_name))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    map_arg = DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map file to load')
    param_arg = DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load')
    use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true')
    
    start_nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_dir,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
    )
    
    declare_x_cmd = DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial x position')
    declare_y_cmd = DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial y position')
    declare_z_cmd = DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Initial z position')
    declare_roll_cmd = DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Initial row position')
    declare_pitch_cmd = DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Initial pitch position')
    declare_yaw_cmd = DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw position')
    
    
    init_pose_publisher = Node(
    package='robotpi_demo_navigation',
    executable='set_initPose',
    name='set_initPose',
    output='screen',
    parameters=[{
      'initial_pose_x': x, 
      'initial_pose_y': y,
      'initial_pose_z': z,
      'initial_pose_row': roll,
      'initial_pose_pitch': pitch,
      'initial_pose_yaw': yaw}])


    return LaunchDescription([
        map_arg,
        param_arg,
        use_sim_time_arg,
        start_nav_bringup,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        init_pose_publisher,
    ])