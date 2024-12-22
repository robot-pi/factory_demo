cmake_minimum_required(VERSION 3.8)
project(ai_manipulate_moveit_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclpy)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_common REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_task_constructor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_task_constructor_core
  moveit_task_constructor_msgs
  rclcpp
  tf2_eigen
)
# Function to add an executable with common settings
function(add_common_executable target_name source_file)
  add_executable(${target_name} src/${source_file})
  target_include_directories(${target_name} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_compile_features(${target_name} PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(${target_name}
    moveit_ros_planning_interface
    rclcpp
    control_msgs
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_task_constructor_core
    moveit_task_constructor_msgs
  )
endfunction()

# Add executables
add_common_executable(arm_manipulate_panda arm_manipulate_panda.cpp)
add_common_executable(gripper_manipulate_panda gripper_manipulate_panda.cpp)
add_common_executable(cartesian_manipulate_panda cartesian_manipulate_panda.cpp)
add_common_executable(cv_to_moveit cv_to_moveit.cpp)
add_common_executable(pose_input_panda pose_input_panda.cpp)
add_common_executable(cartesian_path_command cartesian_path_command.cpp)
add_common_executable(gripper_command_panda gripper_command_panda.cpp)
add_common_executable(mtc_pick_and_place_test mtc_pick_and_place_test.cpp)
add_common_executable(mtc_pick_and_place_cube mtc_pick_and_place_cube.cpp)
# Install targets
install(TARGETS 
arm_manipulate_panda
gripper_manipulate_panda
cartesian_manipulate_panda
cv_to_moveit
pose_input_panda
cartesian_path_command
gripper_command_panda
mtc_pick_and_place_test
mtc_pick_and_place_cube
  DESTINATION lib/${PROJECT_NAME})

# Install directories
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
