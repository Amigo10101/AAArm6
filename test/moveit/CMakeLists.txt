cmake_minimum_required(VERSION 3.22)
project(moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(demo_twist_aar6 src/demo_twist_aar6.cpp)
ament_target_dependencies(demo_twist_aar6
  rclcpp
  moveit_servo
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS demo_twist_aar6
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(servo_keyboard_input_aar6 src/servo_keyboard_input_aar6.cpp)
ament_target_dependencies(servo_keyboard_input_aar6
  rclcpp
  geometry_msgs
  control_msgs
  moveit_msgs
)

install(TARGETS servo_keyboard_input_aar6
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(cartesian_control_aar6 src/cartesian_control_aar6.cpp)
ament_target_dependencies(cartesian_control_aar6
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS cartesian_control_aar6
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
